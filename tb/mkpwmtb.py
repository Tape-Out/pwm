"""pwm 的行为测试台：占空比的三档、互补输出、死区、数组不互相盖。

判据挑成不依赖采样对齐的那种：占空比 0 就一直低、超过周期就一直高、居中就
两种都见得到。数具体多少拍要跟采样相位较劲，换来的确定性还不如这三档。

死区两个方向都验：开着时换向那几拍上下桥必须同时关掉，关着时**从不**同时关掉
——后者才看得出「关了硬件还在」。

认矩阵：`channels` 与 `deadtime` 从这一点的旋钮来。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
chans = int(k.get("channels", 4))
dead = bool(k.get("deadtime", False))

PERIOD = 8
second = chans >= 2

arr_setup = ("      3: wr(8'h14, 32'h00001234);   // duty[1]，验数组不互相盖"
             if second else "      3: noAction;")
arr_check = ('''    if (x.rdata[15:0] != 4) begin
      $display("FAIL duty[0] is %04h, want 0004 (the array aliases)", x.rdata[15:0]);
      wrong = True;
    end''' if second else "    // 只有一路，没有数组可对")

if dead:
    dead_check = '''    if (!sawDead[1]) begin
      $display("FAIL dead time is on but the two sides were never both off");
      wrong = True;
    end'''
    verdict = "duty low, high and mid all behave, and dead time really inserts a gap"
else:
    dead_check = '''    if (sawDead[1]) begin
      $display("FAIL dead time is off but the two sides went off together");
      wrong = True;
    end'''
    verdict = "duty low, high and mid all behave, and the dead time gate really gates"

txt = f'''package Pwm{label}Tb;

import RegIf::*;
import Pwm::*;

// 由 tb/mkpwmtb.py 生成，勿手改。这一点：channels={chans} deadtime={dead}

typedef enum {{ Setup, Low, SetHigh, High, SetMid, Mid, Check, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkPwm{label}Tb(Empty);
  PwmIfc#(8, 32, {chans}) d <- mkPwm(
      PwmCfg {{ deadtime: {"True" if dead else "False"} }});

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  // 采样规则每拍都跑，凡是它写、检查规则读的量都得用 CReg
  Reg#(Bool) sawHi[2]   <- mkCReg(2, False);
  Reg#(Bool) sawLo[2]   <- mkCReg(2, False);
  Reg#(Bool) sawDead[2] <- mkCReg(2, False);

  rule sample;
    Bit#({chans}) o  = d.pins.out;
    Bit#({chans}) on = d.pins.outn;
    if (o[0] == 1) sawHi[0] <= True;
    if (o[0] == 0) sawLo[0] <= True;
    // 上下桥同时关掉，只有死区里才该出现
    if (o[0] == 0 && on[0] == 0) sawDead[0] <= True;
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 40000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: v, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Setup);
    case (s)
      0: wr(8'h04, {PERIOD});          // period
      1: wr(8'h10, 0);             // duty[0] = 0，该一直低
      2: wr(8'h40, 3);             // dead，特性关掉时写了也不算数
{arr_setup}
      4: wr(8'h00, 32'h00000001);  // en，presc = 0，边沿对齐
      default: begin ph <= Low; sawHi[1] <= False; sawLo[1] <= False; end
    endcase
    if (s < 5) s <= s + 1; else s <= 0;
  endrule

  // 占空比 0：数满一整个周期都不该有高电平
  rule low (ph == Low);
    if (s > {PERIOD * 3}) begin
      if (sawHi[1]) begin
        $display("FAIL duty is zero but the output went high");
        bad <= True;
      end
      ph <= SetHigh;
      s  <= 0;
    end else s <= s + 1;
  endrule

  rule setHigh (ph == SetHigh);
    if (s == 0) wr(8'h10, {PERIOD + 1});   // 占空比超过周期
    // 换向那一下死区会把两路都关掉几拍，那是它本来的行为。等它过去再开始看，
    // 否则量到的是死区而不是占空比。
    if (s == 6) begin sawLo[1] <= False; sawHi[1] <= False; end
    if (s == 7) begin ph <= High; s <= 0; end
    else s <= s + 1;
  endrule

  rule high (ph == High);
    if (s > {PERIOD * 3}) begin
      if (sawLo[1]) begin
        $display("FAIL duty covers the whole period but the output went low");
        bad <= True;
      end
      ph <= SetMid;
      s  <= 0;
    end else s <= s + 1;
  endrule

  rule setMid (ph == SetMid);
    if (s == 0) wr(8'h10, 4);      // 居中，两种都该见到
    if (s == 6) begin
      sawLo[1] <= False; sawHi[1] <= False; sawDead[1] <= False;
    end
    if (s == 7) begin ph <= Mid; s <= 0; end
    else s <= s + 1;
  endrule

  rule mid (ph == Mid);
    if (s > {PERIOD * 4}) begin ph <= Check; s <= 0; end
    else s <= s + 1;
  endrule

  rule check (ph == Check);
    let x <- d.regs.access(RegReq {{ addr: 8'h10, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (!sawHi[1] || !sawLo[1]) begin
      $display("FAIL a mid duty did not toggle: high=%0d low=%0d",
               sawHi[1], sawLo[1]);
      wrong = True;
    end
{dead_check}
{arr_check}
    if (wrong) bad <= True;
    ph <= Done;
  endrule

  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS pwm: {verdict}");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"Pwm{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  pwm 行为测试台就位：channels={chans} deadtime={dead}")
