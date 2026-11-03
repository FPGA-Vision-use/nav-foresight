// rollout 核 v1：43 候选 × 400 步，纯 SI 单位（m, m/s, rad），输出 int16 毫米点云。
//
// 三个改写让 43 路铺得下：
//   1. sin/cos(h+turn/2) → 增量旋转 (cosh,sinh) × 小角旋转矩阵
//   2. sinc(turn/2π)     → chord ≈ arc
//   3. 三处 /vn           → 每步一次 inv_v，其余变乘法
#include <ap_int.h>

#define K 43
#define N 400
#define NBEAT (N / 4)
#define DT 0.02f
#define BIG 1e30f

// 控制档由 k 直接算出，顺序与 controls_bank() 严格一致：
//   k=0 直行/刹停；k>=1 时 j=k-1，每 6 个一组：
//   grp=j/6 -> peak=-1.5+0.5*grp；sub=j%6 -> lane=sub/3, sp=sub%3
//   sp=0 -> (0.15, 8s)  sp=1 -> (0.30, 8s)  sp=2 -> (0.30, 3s)
struct Opt { float peak; float tgt; float stop; int lane; };

static Opt decode(int k) {
  Opt o;
  o.peak = 0.f; o.tgt = 0.f; o.stop = 0.f; o.lane = 0;
  if (k > 0) {
    int j = k - 1;
    o.peak = -1.5f + 0.5f * (float)(j / 6);
    o.lane = (j % 6) / 3;
    int sp = j % 3;
    o.tgt = (sp == 0) ? 0.15f : 0.3f;
    o.stop = (sp == 2) ? 3.f : 8.f;
  }
  return o;
}

static float fclamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void rollout(float x0, float y0, float ch0, float sh0, float v0, float k0,
             ap_uint<128>* pts) {
#pragma HLS INTERFACE m_axi port = pts offset = slave bundle = gmem depth = 4311 max_write_burst_length = 16
#pragma HLS INTERFACE s_axilite port = x0 bundle = control
#pragma HLS INTERFACE s_axilite port = y0 bundle = control
#pragma HLS INTERFACE s_axilite port = ch0 bundle = control
#pragma HLS INTERFACE s_axilite port = sh0 bundle = control
#pragma HLS INTERFACE s_axilite port = v0 bundle = control
#pragma HLS INTERFACE s_axilite port = k0 bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

  ap_uint<32> fl[K];
#pragma HLS ARRAY_PARTITION variable = fl complete
  for (int k = 0; k < K; k++) {
    Opt o = decode(k);
    float x = x0, y = y0, v = v0, a = 0.f, kk = k0;   // 初值曲率必须由 PS 给
    float ch = ch0, sh = sh0;   // 初值必须来自真实航向
    bool ok = true;
    for (int b = 0; b < NBEAT; b++) {
      ap_uint<128> w = 0;
    for (int q = 0; q < 4; q++) {
#pragma HLS UNROLL
      int i = b * 4 + q;
      float t = i * DT;
      // 纵向：jerk 限幅 + 目标速度
      float dv = (t < o.stop) ? o.tgt : 0.f;
      float lo = (-0.4f - a) / DT;
      float hi = (0.4f - a) / DT;
      float lo2 = -2.f * (v + a * DT) / (DT * DT);
      float hi2 = 2.f * (0.5f - v - a * DT) / (DT * DT);
      if (lo2 > lo) lo = lo2;
      if (hi2 < hi) hi = hi2;
      if (-0.8f > lo) lo = -0.8f;
      if (0.8f < hi) hi = 0.8f;
      if (lo > hi) ok = false;
      float j = fclamp(16.f * (dv - v) - 8.f * a, lo, hi);
      float an = a + j * DT;
      float vn = v + a * DT + .5f * j * DT * DT;
      // 横向：曲率限幅（三处 /vn 收成一次倒数）
      float vm = vn > 0.02f ? vn : 0.02f;
      float inv_v = 1.f / vm;
      float inv_v2 = inv_v * inv_v;
      float dk = o.peak;
      if (t >= 3.f) dk = (o.lane && t < 6.f) ? -o.peak : 0.f;
      float cap = 1.5f;
      float c2 = 0.8f * inv_v;
      float c3 = 0.3f * inv_v2;
      if (c2 < cap) cap = c2;
      if (c3 < cap) cap = c3;
      dk = fclamp(dk, -cap, cap);
      float abs_ak = (an > 0 ? an : -an) * (kk > 0 ? kk : -kk);
      float rc = (1.5f - abs_ak) * inv_v;
      if (rc < 0.f) rc = 0.f;
      if (rc > 0.8f) rc = 0.8f;
      float rate = fclamp((dk - kk) / .45f, -rc, rc);
      float kn = kk + rate * DT;
      // 推进：增量旋转，无三角函数
      float arc = .5f * (v + vn) * DT;
      float turn = .5f * (kk + kn) * arc;
      float half = turn * .5f;
      x += arc * (ch - sh * half);
      y += arc * (sh + ch * half);
      float ct = 1.f - turn * turn * .5f;
      float st = turn - turn * turn * turn / 6.f;
      float chn = ch * ct - sh * st;
      float shn = sh * ct + ch * st;
      ch = chn;   // 400 步累计模长漂移 ~7e-5，不需归一化
      sh = shn;
      v = vn; a = an; kk = kn;
      int xi = (int)(x * 1000.f);
      int yi = (int)(y * 1000.f);
      if (xi > 32767) xi = 32767;
      if (xi < -32768) xi = -32768;
      if (yi > 32767) yi = 32767;
      if (yi < -32768) yi = -32768;
      w.range(32 * q + 31, 32 * q) =
          (ap_uint<32>)(((unsigned)(yi & 0xFFFF) << 16) | (unsigned)(xi & 0xFFFF));
    }
    pts[k * NBEAT + b] = w;
    }
    fl[k] = ok ? 1u : 0u;
  }
  // 43 个可行标志按 int32 布局拼进 pts 尾部（每 128 位字放 4 个）
  for (int b = 0; b < (K + 3) / 4; b++) {
#pragma HLS PIPELINE II = 1
    ap_uint<128> w = 0;
    for (int q = 0; q < 4; q++) {
#pragma HLS UNROLL
      int kk = b * 4 + q;
      if (kk < K) w.range(32 * q + 31, 32 * q) = fl[kk];
    }
    pts[K * NBEAT + b] = w;
  }
}
