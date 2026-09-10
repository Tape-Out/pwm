"""pwm 的行为测试台：占空比的三档、互补输出、死区、数组不互相盖。

判据挑成不依赖采样对齐的那种：占空比 0 就一直低、超过周期就一直高、居中就
两种都见得到。数具体多少拍要跟采样相位较劲，换来的确定性还不如这三档。

死区两个方向都验：开着时换向那几拍上下桥必须同时关掉，关着时**从不**同时关掉
——后者才看得出「关了硬件还在」。

影子也两个方向都验：默认（`imm=0`）在脉冲中途改占空比不许削掉当前这一个脉冲，
置上 `imm` 之后必须当场生效。只验一头的话，把影子接死或者把 `imm` 忽略掉都测不出来。

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
SETTLE = 2 * PERIOD + 4   # 影子要等一个周期边界，歇够一整圈再看
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
    verdict = "duty low, high and mid all behave, the shadow holds, and dead time inserts a gap"
else:
    dead_check = '''    if (sawDead[1]) begin
      $display("FAIL dead time is off but the two sides went off together");
      wrong = True;
    end'''
    verdict = "duty low, high and mid all behave, the shadow holds, and the dead time gate gates"

txt = f'''package Pwm{label}Tb;

import RegIf::*;
import Pwm::*;

// 由 tb/mkpwmtb.py 生成，勿手改。这一点：channels={chans} deadtime={dead}

typedef enum {{ Setup, Low, SetHigh, High, SetMid, Mid, Check,
               SetGlit, ArmGlit, ChkGlit, SetImm, ArmImm, ChkImm,
               SetZero, CheckZero, Done }}
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

  // 上升沿单独认一次：影子那两条判据要正踩在脉冲开头写新占空比，
  // 落在别处就分不出「脉冲被削」与「本来就该低」
  Reg#(Bit#(1)) prevO <- mkReg(0);
  PulseWire     rose  <- mkPulseWire;

  rule edge_;
    Bit#({chans}) o = d.pins.out;
    prevO <= o[0];
    if (o[0] == 1 && prevO == 0) rose.send;
  endrule

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
    if (s == {SETTLE}) begin sawLo[1] <= False; sawHi[1] <= False; end
    if (s == {SETTLE + 1}) begin ph <= High; s <= 0; end
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
    if (s == {SETTLE}) begin
      sawLo[1] <= False; sawHi[1] <= False; sawDead[1] <= False;
    end
    if (s == {SETTLE + 1}) begin ph <= Mid; s <= 0; end
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
    ph <= SetGlit;
    s  <= 0;
  endrule

  // 周期中途改占空比：默认该等到这一周期数完。半桥驱动里当场生效会把已经开着
  // 的上桥立刻关掉，削出一个既不是旧占空比也不是新占空比的畸形脉冲。
  // 死区先关掉——它换向时本来就会把两路都压低几拍，会盖住要看的东西。
  rule setGlit (ph == SetGlit);
    case (s)
      0: wr(8'h40, 0);              // dead = 0
      1: wr(8'h10, 6);              // 占空比 6/8，脉冲够长，写得进中途
      default: noAction;
    endcase
    if (s > {SETTLE + 2}) begin ph <= ArmGlit; s <= 0; end
    else s <= s + 1;
  endrule

  rule armGlit (ph == ArmGlit);
    if (rose) begin wr(8'h10, 0); ph <= ChkGlit; s <= 0; end
  endrule

  rule chkGlit (ph == ChkGlit);
    if (s == 2 && d.pins.out[0] == 0) begin
      $display("FAIL a duty write mid pulse cut the pulse short: no shadow");
      bad <= True;
    end
    if (s > 3) begin ph <= SetImm; s <= 0; end
    else s <= s + 1;
  endrule

  // 置上 imm 就该当场生效——闭环电流环等不起一个周期
  rule setImm (ph == SetImm);
    case (s)
      0: wr(8'h10, 6);
      1: wr(8'h00, 32'h00000005);   // en + imm
      default: noAction;
    endcase
    if (s > {SETTLE + 2}) begin ph <= ArmImm; s <= 0; end
    else s <= s + 1;
  endrule

  rule armImm (ph == ArmImm);
    if (rose) begin wr(8'h10, 0); ph <= ChkImm; s <= 0; end
  endrule

  rule chkImm (ph == ChkImm);
    if (s == 2 && d.pins.out[0] == 1) begin
      $display("FAIL imm is set but the duty write still waited for the boundary");
      bad <= True;
    end
    if (s > 3) begin ph <= SetZero; s <= 0; end
    else s <= s + 1;
  endrule

  // 中心对齐 + period = 0：退化成一拍的周期，计数器该停在 0，输出跟着占空比走。
  // 上下计数若写成「到顶就减一」，0 - 1 会绕成 65535，整整六万多拍全错。
  // period 的复位值就是 0，所以「先使能、后配周期」正好踩中。
  rule setZero (ph == SetZero);
    case (s)
      0: wr(8'h00, 0);              // 先关
      1: wr(8'h04, 0);              // period = 0
      2: wr(8'h10, 1);              // duty[0] = 1
      3: wr(8'h00, 32'h00000003);   // en + align（中心对齐）
      default: noAction;
    endcase
    if (s > 8) begin ph <= CheckZero; s <= 0; end
    else s <= s + 1;
  endrule

  rule checkZero (ph == CheckZero);
    // 换向那一下死区会把两路都关掉几拍，等它过去再开始看
    if (s == 12) sawLo[1] <= False;
    if (s > 120) begin
      if (sawLo[1]) begin
        $display("FAIL center aligned with a zero period: the output fell, the counter wrapped");
        bad <= True;
      end
      ph <= Done;
    end else s <= s + 1;
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
