// 净空核 v1 —— 五类 margin，8 路并行，256 位输入，先全部读进片上 BRAM 再多遍扫描。
// 语义见 INTERFACE.md：毫米整数、点语义（线段差值已实测 <=15 mm）、时间半径取 max(r_i,r_{i+1})。
#include <ap_int.h>

#define K 43
#define N 400
#define LANES 8
#define NBEAT (N / LANES)          // 50
#define TOTAL (K * NBEAT)          // 2150 个 256 位字
#define BIG 1073741824
#define NS 7                       // sweep tau 数
#define NC 11                      // cloud tau 数

static int isqrt32(unsigned int v) {
  // 拆成两段 8 级：中间那组变量会被 HLS 打一拍，关键路径从 16 级降到 8 级
  unsigned int rem = 0u, root = 0u;
  for (int i = 15; i >= 8; i--) {
#pragma HLS UNROLL
    root <<= 1;
    rem = (rem << 2) | ((v >> (2 * i)) & 3u);
    if (root < rem) { rem -= root + 1u; root += 2u; }
  }
  unsigned int rem_hi = rem & 0xFFFFu;
  unsigned int root_hi = root & 0xFFFFu;
  rem = rem_hi;
  root = root_hi;
  for (int i = 7; i >= 0; i--) {
#pragma HLS UNROLL
    root <<= 1;
    rem = (rem << 2) | ((v >> (2 * i)) & 3u);
    if (root < rem) { rem -= root + 1u; root += 2u; }
  }
  return (int)(root >> 1);
}

static int satd(int d) { return d > 10000 ? 10000 : (d < -10000 ? -10000 : d); }

// clamp 后 |d| <= 10000，15 位足够：用 16 位乘法代替 32x34 位，关键路径从 11ns 降到 ~5ns
static int sq(int d) { ap_int<16> a = d; return (int)a * (int)a; }

// 一次扫描：所有候选对一个固定中心，返回 43 个 (dist - radius) 的最小值
static void pass_fixed(ap_uint<256> buf[TOTAL], int cx, int cy, int radius,
                       int out[K]) {
  for (int k = 0; k < K; k++) {
    int best = BIG;
    for (int b = 0; b < NBEAT; b++) {
#pragma HLS PIPELINE II = 3
      ap_uint<256> w = buf[k * NBEAT + b];
      int d[LANES];
#pragma HLS ARRAY_PARTITION variable = d complete
      for (int q = 0; q < LANES; q++) {
#pragma HLS UNROLL
        ap_int<32> pw = w.range(q * 32 + 31, q * 32);
        int dx = satd((short)((int)pw & 0xffff) - cx);
        int dy = satd((short)((int)pw >> 16) - cy);
        d[q] = isqrt32((unsigned)(sq(dx) + sq(dy)));
      }
      for (int q = 0; q < LANES; q++) {
#pragma HLS UNROLL
        int g = d[q] - radius;
        best = (g < best) ? g : best;
      }
    }
    out[k] = best;
  }
}

void clearance(const ap_uint<256>* pts,
               // 参数：毫米 / 毫米每秒
               int xmin, int ymin, int xmax, int ymax,
               int pcx, int pcy, int psx, int psy,
               int acx, int acy, int avx, int avy, int abase,
               int ccx, int ccy, int cvx, int cvy, int crad,
               int margin_mm, int sw_guard, int* out /* K*5 */) {
#pragma HLS INTERFACE m_axi port = pts offset = slave bundle = gmem depth = 2150
#pragma HLS INTERFACE m_axi port = out offset = slave bundle = gmem depth = 215
#pragma HLS INTERFACE s_axilite port = xmin bundle = control
#pragma HLS INTERFACE s_axilite port = ymin bundle = control
#pragma HLS INTERFACE s_axilite port = xmax bundle = control
#pragma HLS INTERFACE s_axilite port = ymax bundle = control
#pragma HLS INTERFACE s_axilite port = pcx bundle = control
#pragma HLS INTERFACE s_axilite port = pcy bundle = control
#pragma HLS INTERFACE s_axilite port = psx bundle = control
#pragma HLS INTERFACE s_axilite port = psy bundle = control
#pragma HLS INTERFACE s_axilite port = acx bundle = control
#pragma HLS INTERFACE s_axilite port = acy bundle = control
#pragma HLS INTERFACE s_axilite port = avx bundle = control
#pragma HLS INTERFACE s_axilite port = avy bundle = control
#pragma HLS INTERFACE s_axilite port = abase bundle = control
#pragma HLS INTERFACE s_axilite port = ccx bundle = control
#pragma HLS INTERFACE s_axilite port = ccy bundle = control
#pragma HLS INTERFACE s_axilite port = cvx bundle = control
#pragma HLS INTERFACE s_axilite port = cvy bundle = control
#pragma HLS INTERFACE s_axilite port = crad bundle = control
#pragma HLS INTERFACE s_axilite port = margin_mm bundle = control
#pragma HLS INTERFACE s_axilite port = sw_guard bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

  // 1) 一次性把 68.8 KB 读进片上
  ap_uint<256> buf[TOTAL];
#pragma HLS ARRAY_PARTITION variable = buf cyclic factor = 1 dim = 1
  for (int i = 0; i < TOTAL; i++) {
#pragma HLS PIPELINE II = 3
    buf[i] = pts[i];
  }

  int m_bound[K], m_pil[K], m_timed[K], m_sweep[NS][K], m_cloud[NC][K];
#pragma HLS ARRAY_PARTITION variable = m_sweep complete dim = 2
#pragma HLS ARRAY_PARTITION variable = m_cloud complete dim = 2
#pragma HLS ARRAY_PARTITION variable = m_bound complete
#pragma HLS ARRAY_PARTITION variable = m_pil complete
#pragma HLS ARRAY_PARTITION variable = m_timed complete

  // 2) boundary（不开方）
  for (int k = 0; k < K; k++) {
    int best = BIG;
    for (int b = 0; b < NBEAT; b++) {
#pragma HLS PIPELINE II = 3
      ap_uint<256> w = buf[k * NBEAT + b];
      for (int q = 0; q < LANES; q++) {
#pragma HLS UNROLL
        ap_int<32> pw = w.range(q * 32 + 31, q * 32);
        int x = (short)((int)pw & 0xffff);
        int y = (short)((int)pw >> 16);
        int v0 = x - (xmin + margin_mm);
        int v1 = (xmax - margin_mm) - x;
        int v2 = y - (ymin + margin_mm);
        int v3 = (ymax - margin_mm) - y;
        int v = v0 < v1 ? v0 : v1;
        v = v < v2 ? v : v2;
        v = v < v3 ? v : v3;
        best = (v < best) ? v : best;
      }
    }
    m_bound[k] = best;
  }

  // 3) pillar（轴对齐盒 -> 开方）
  for (int k = 0; k < K; k++) {
    int best = BIG;
    for (int b = 0; b < NBEAT; b++) {
#pragma HLS PIPELINE II = 3
      ap_uint<256> w = buf[k * NBEAT + b];
      for (int q = 0; q < LANES; q++) {
#pragma HLS UNROLL
        ap_int<32> pw = w.range(q * 32 + 31, q * 32);
        int dx = ((short)((int)pw & 0xffff) > pcx ? (short)((int)pw & 0xffff) - pcx
                                                   : pcx - (short)((int)pw & 0xffff)) -
                 psx / 2;
        int dy = ((short)((int)pw >> 16) > pcy ? (short)((int)pw >> 16) - pcy
                                               : pcy - (short)((int)pw >> 16)) -
                 psy / 2;
        dx = dx > 0 ? dx : 0;
        dy = dy > 0 ? dy : 0;
        int d = isqrt32((unsigned)(sq(dx) + sq(dy)));
        int g = d - margin_mm - 5;
        best = (g < best) ? g : best;
      }
    }
    m_pil[k] = best;
  }

  // 4) actor timed：中心随 t 移动，半径随 t 增长，取两拍较大半径
  for (int k = 0; k < K; k++) {
    int best = BIG;
    for (int b = 0; b < NBEAT; b++) {
#pragma HLS PIPELINE II = 3
      ap_uint<256> w = buf[k * NBEAT + b];
      for (int q = 0; q < LANES; q++) {
#pragma HLS UNROLL
        int i = b * LANES + q;
        int cx = acx + (avx * i * 20) / 1000;   // v[mm/s] * t[s]
        int cy = acy + (avy * i * 20) / 1000;
        int r = abase + (36 * i) / 10 + (6 * i * i) / 1000;
        int r2 = abase + (36 * (i + 1)) / 10 + (6 * (i + 1) * (i + 1)) / 1000;
        r = r > r2 ? r : r2;
        ap_int<32> pw = w.range(q * 32 + 31, q * 32);
        int dx = satd((short)((int)pw & 0xffff) - cx);
        int dy = satd((short)((int)pw >> 16) - cy);
        int d = isqrt32((unsigned)(sq(dx) + sq(dy)));
        int g = d - r;
        best = (g < best) ? g : best;
      }
    }
    m_timed[k] = best;
  }

  // 5) actor sweep: NS 个固定中心
  for (int s = 0; s < NS; s++) {
    int tau100 = (s * 120) / (NS - 1);           // tau*100，0..120
    int cx = acx + (avx * tau100) / 100;
    int cy = acy + (avy * tau100) / 100;
    // 350 必须来自运行时参数：写成常量时 HLS 会把 "350 + tau100" 收窄成 9 位有符号
    // 中间量，综合时按符号扩展折叠（350→-162），sweep 列整列少 512 mm。
    // 板上实测 abase=520/1032/8 三点恒定偏 +512，根因即此。
    int r = abase + sw_guard + tau100;
    int tmp[K];
    pass_fixed(buf, cx, cy, r, tmp);
    for (int k = 0; k < K; k++) m_sweep[s][k] = tmp[k];
  }

  // 6) cloud: NC 个固定中心
  for (int s = 0; s < NC; s++) {
    int tau100 = (s * 100) / (NC - 1);           // tau*100，0..100
    int cx = ccx + (cvx * tau100) / 100;
    int cy = ccy + (cvy * tau100) / 100;
    int r = crad + (12 * tau100) / 10;
    int tmp[K];
    pass_fixed(buf, cx, cy, r, tmp);
    for (int k = 0; k < K; k++) m_cloud[s][k] = tmp[k];
  }

  // 7) 汇合：每个候选 5 个数
  for (int k = 0; k < K; k++) {
    int sw = m_sweep[0][k];
    for (int s = 1; s < NS; s++) sw = m_sweep[s][k] < sw ? m_sweep[s][k] : sw;
    int cl = m_cloud[0][k];
    for (int s = 1; s < NC; s++) cl = m_cloud[s][k] < cl ? m_cloud[s][k] : cl;
    out[k * 5 + 0] = m_bound[k];
    out[k * 5 + 1] = m_pil[k];
    out[k * 5 + 2] = m_timed[k];
    out[k * 5 + 3] = sw;
    out[k * 5 + 4] = cl;
  }
}
