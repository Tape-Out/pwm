package Pwm;

import Vector::*;
import RegIf::*;
import PwmRegs::*;

// 恒零的只读寄存器：特性关掉时占位，写进去什么也不发生，综合器整片消掉
function Reg#(t) roReg(t v) =
  interface Reg;
    method t _read = v;
    method Action _write(t x) = noAction;
  endinterface;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool deadtime;
} PwmCfg;

interface PwmPins#(numeric type channels);
  (* always_ready, result = "pwm"   *) method Bit#(channels) out;
  (* always_ready, result = "pwm_n" *) method Bit#(channels) outn;
endinterface

interface PwmIfc#(numeric type aw, numeric type dw, numeric type channels);
  interface RegIf#(aw, dw) regs;
  interface PwmPins#(channels) pins;
endinterface

module mkPwm#(PwmCfg cfg)(PwmIfc#(aw, dw, channels))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 16, dw),
              Add#(_c, 8, dw), Add#(_d, 1, dw));

  PwmRegsIfc#(aw, dw, channels) r <- mkPwmRegs(
      PwmRegsCfg { deadtime: cfg.deadtime });

  Reg#(Bit#(16)) cnt  <- mkReg(0);
  Reg#(Bit#(16)) div  <- mkReg(0);
  Reg#(Bool)     down <- mkReg(False);   // 中心对齐时的方向

  rule tick (r.ctrl_en == 1);
    if (div >= r.ctrl_presc) begin
      div <= 0;
      if (r.ctrl_align == 1) begin
        // 中心对齐：来回数，边沿以周期中点为轴对称，互补驱动不会同时导通
        if (down) begin
          if (cnt == 0) begin down <= False; cnt <= 1; end
          else cnt <= cnt - 1;
        end else begin
          if (cnt >= r.period) begin down <= True; cnt <= cnt - 1; end
          else cnt <= cnt + 1;
        end
      end else
        cnt <= (cnt >= r.period) ? 0 : cnt + 1;
    end else
      div <= div + 1;
  endrule

  function Bit#(channels) rawOut();
    Bit#(channels) o = 0;
    for (Integer i = 0; i < valueOf(channels); i = i + 1)
      if (r.ctrl_en == 1 && cnt < r.duty[i]) o[i] = 1;
    return o;
  endfunction

  // 关掉死区就真的不例化这些寄存器。只挡规则是不够的——模块还在，
  // 综合器留着它的复位逻辑，特性看起来免费其实一直在付钱（hart 的 M 扩展
  // 就是这么量出 98 µm² 这个假数的）。
  Vector#(channels, Reg#(Bit#(8))) dz = replicate(roReg(0));
  Reg#(Bit#(channels))             prev = roReg(0);

  if (cfg.deadtime) begin
    dz   <- replicateM(mkReg(0));
    prev <- mkReg(0);
    // 换向的一瞬两路都关掉，关满 dead 拍再放行。上下桥直通烧管子就是这么来的。
    rule guard;
      let raw = rawOut();
      prev <= raw;
      for (Integer i = 0; i < valueOf(channels); i = i + 1)
        if (raw[i] != prev[i]) dz[i] <= r.dead;
        else if (dz[i] != 0) dz[i] <= dz[i] - 1;
    endrule
  end

  function Bool live(Integer i) = !cfg.deadtime || dz[i] == 0;

  interface regs = r.regs;
  interface PwmPins pins;
    method Bit#(channels) out;
      Bit#(channels) o = 0;
      let raw = rawOut();
      for (Integer i = 0; i < valueOf(channels); i = i + 1)
        if (raw[i] == 1 && live(i)) o[i] = 1;
      return o;
    endmethod
    method Bit#(channels) outn;
      Bit#(channels) o = 0;
      let raw = rawOut();
      for (Integer i = 0; i < valueOf(channels); i = i + 1)
        if (raw[i] == 0 && live(i) && r.ctrl_en == 1) o[i] = 1;
      return o;
    endmethod
  endinterface
endmodule

endpackage
