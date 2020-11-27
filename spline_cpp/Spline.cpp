//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Spline.cpp
//
//  Code generation for function 'Spline'
//


// Include files
#include "Spline.h"

// Function Declarations
static int div_s32(int numerator, int denominator);

// Function Definitions
Spline::~Spline()
{
  // (no terminate code required)
}

Spline::Spline()
{
}

void Spline::spline_cpp(const coder::array<double, 2U> &x, const coder::array<
  double, 2U> &y, coder::array<double, 2U> &coefs)
{
  int nx;
  boolean_T has_endslopes;
  int szdvdf_idx_1;
  double d1;
  coder::array<double, 2U> s;
  coder::array<double, 2U> dvdf;
  double c_data[8];
  coder::array<double, 2U> dx;
  coder::array<double, 3U> pp1_coefs;
  coder::array<double, 3U> pp_coefs;
  int i;
  coder::array<double, 2U> md;
  nx = x.size(1) - 1;
  has_endslopes = (y.size(1) == x.size(1) + 2);
  if ((x.size(1) <= 2) || ((x.size(1) <= 3) && (!has_endslopes))) {
    int szs_idx_1;
    has_endslopes = (y.size(1) == x.size(1) + 2);
    if (x.size(1) <= 2) {
      if (has_endslopes) {
        szs_idx_1 = 4;
      } else {
        szs_idx_1 = 2;
      }

      if (has_endslopes) {
        int nxm1;
        double d31;
        double endslopes_idx_1;
        szdvdf_idx_1 = (y.size(1) - 1) << 1;
        d31 = y[0];
        endslopes_idx_1 = y[1];
        nxm1 = x.size(1);
        nx = (x.size(1) - 1) << 1;
        pp1_coefs.set_size(2, (x.size(1) - 1), 4);
        for (int j = 0; j <= nxm1 - 2; j++) {
          double dxj;
          double dnnm2;
          dxj = x[1] - x[0];
          d1 = (y[4] - y[2]) / dxj;
          dnnm2 = (d1 - d31) / dxj;
          d1 = (y[szdvdf_idx_1] - d1) / dxj;
          pp1_coefs[0] = (d1 - dnnm2) / dxj;
          pp1_coefs[nx] = 2.0 * dnnm2 - d1;
          i = nx << 1;
          pp1_coefs[i] = d31;
          pp1_coefs[3 * nx] = y[2];
          d1 = (y[5] - y[3]) / dxj;
          dnnm2 = (d1 - endslopes_idx_1) / dxj;
          d1 = (y[szdvdf_idx_1 + 1] - d1) / dxj;
          pp1_coefs[1] = (d1 - dnnm2) / dxj;
          pp1_coefs[nx + 1] = 2.0 * dnnm2 - d1;
          pp1_coefs[i + 1] = endslopes_idx_1;
          pp1_coefs[3 * nx + 1] = y[3];
        }

        szdvdf_idx_1 = 2 * szs_idx_1;
        for (i = 0; i < szdvdf_idx_1; i++) {
          c_data[i] = pp1_coefs[i];
        }
      } else {
        d1 = x[1] - x[0];
        c_data[0] = (y[2] - y[0]) / d1;
        c_data[2] = y[0];
        c_data[1] = (y[3] - y[1]) / d1;
        c_data[3] = y[1];
      }
    } else {
      double d31;
      double dxj;
      double dnnm2;
      szs_idx_1 = 3;
      d31 = x[2] - x[0];
      d1 = x[1] - x[0];
      dxj = x[2] - x[1];
      dnnm2 = (y[2] - y[0]) / d1;
      c_data[0] = ((y[4] - y[2]) / dxj - dnnm2) / d31;
      c_data[2] = dnnm2 - c_data[0] * d1;
      c_data[4] = y[0];
      dnnm2 = (y[3] - y[1]) / d1;
      c_data[1] = ((y[5] - y[3]) / dxj - dnnm2) / d31;
      c_data[3] = dnnm2 - c_data[1] * d1;
      c_data[5] = y[1];
    }

    pp_coefs.set_size(2, 1, szs_idx_1);
    szdvdf_idx_1 = 2 * szs_idx_1;
    for (i = 0; i < szdvdf_idx_1; i++) {
      pp_coefs[i] = c_data[i];
    }
  } else {
    int nxm1;
    int szs_idx_1;
    double d31;
    int yoffset;
    double dxj;
    double dnnm2;
    int k;
    int pg;
    nxm1 = x.size(1) - 1;
    if (has_endslopes) {
      szdvdf_idx_1 = y.size(1) - 3;
      szs_idx_1 = y.size(1) - 2;
      yoffset = 1;
    } else {
      szdvdf_idx_1 = y.size(1) - 1;
      szs_idx_1 = y.size(1);
      yoffset = -1;
    }

    s.set_size(2, szs_idx_1);
    dvdf.set_size(2, szdvdf_idx_1);
    dx.set_size(1, (x.size(1) - 1));
    for (k = 0; k < nxm1; k++) {
      dx[k] = x[k + 1] - x[k];
      szdvdf_idx_1 = k << 1;
      pg = (yoffset + szdvdf_idx_1) + 1;
      szs_idx_1 = (yoffset + ((k + 1) << 1)) + 1;
      dvdf[szdvdf_idx_1] = (y[szs_idx_1] - y[pg]) / dx[k];
      dvdf[szdvdf_idx_1 + 1] = (y[szs_idx_1 + 1] - y[pg + 1]) / dx[k];
    }

    for (k = 2; k <= nxm1; k++) {
      pg = ((k - 1) << 1) - 1;
      szs_idx_1 = (k - 2) << 1;
      dxj = dx[k - 1];
      d1 = dx[k - 2];
      s[pg + 1] = 3.0 * (dxj * dvdf[szs_idx_1] + d1 * dvdf[pg + 1]);
      s[pg + 2] = 3.0 * (dxj * dvdf[szs_idx_1 + 1] + d1 * dvdf[pg + 2]);
    }

    if (has_endslopes) {
      d31 = 0.0;
      dnnm2 = 0.0;
      s[0] = dx[1] * y[0];
      s[1] = dx[1] * y[1];
      szdvdf_idx_1 = (x.size(1) + 1) << 1;
      szs_idx_1 = (x.size(1) - 1) << 1;
      s[szs_idx_1] = dx[nx - 2] * y[szdvdf_idx_1];
      s[szs_idx_1 + 1] = dx[nx - 2] * y[szdvdf_idx_1 + 1];
    } else {
      d31 = x[2] - x[0];
      dnnm2 = x[x.size(1) - 1] - x[x.size(1) - 3];
      d1 = dx[0];
      dxj = (d1 + 2.0 * d31) * dx[1];
      d1 *= d1;
      s[0] = (dxj * dvdf[0] + d1 * dvdf[2]) / d31;
      s[1] = (dxj * dvdf[1] + d1 * dvdf[3]) / d31;
      pg = (x.size(1) - 1) << 1;
      szs_idx_1 = (x.size(1) - 2) << 1;
      szdvdf_idx_1 = (x.size(1) - 3) << 1;
      d1 = dx[x.size(1) - 2];
      dxj = d1 + 2.0 * dnnm2;
      d1 *= d1;
      s[pg] = (dxj * dx[x.size(1) - 3] * dvdf[szs_idx_1] + d1 *
               dvdf[szdvdf_idx_1]) / dnnm2;
      s[pg + 1] = (dxj * dx[x.size(1) - 3] * dvdf[szs_idx_1 + 1] + d1 *
                   dvdf[szdvdf_idx_1 + 1]) / dnnm2;
    }

    md.set_size(x.size(0), x.size(1));
    md[0] = dx[1];
    md[x.size(1) - 1] = dx[x.size(1) - 3];
    for (k = 2; k <= nxm1; k++) {
      md[k - 1] = 2.0 * (dx[k - 1] + dx[k - 2]);
    }

    d1 = dx[1] / md[0];
    md[1] = md[1] - d1 * d31;
    s[2] = s[2] - d1 * s[0];
    s[3] = s[3] - d1 * s[1];
    for (k = 3; k <= nxm1; k++) {
      d1 = dx[k - 1] / md[k - 2];
      md[k - 1] = md[k - 1] - d1 * dx[k - 3];
      pg = ((k - 1) << 1) - 1;
      szs_idx_1 = (k - 2) << 1;
      s[pg + 1] = s[pg + 1] - d1 * s[szs_idx_1];
      s[pg + 2] = s[pg + 2] - d1 * s[szs_idx_1 + 1];
    }

    d1 = dnnm2 / md[x.size(1) - 2];
    md[x.size(1) - 1] = md[x.size(1) - 1] - d1 * dx[x.size(1) - 3];
    pg = ((x.size(1) - 1) << 1) - 1;
    szs_idx_1 = (x.size(1) - 2) << 1;
    s[pg + 1] = s[pg + 1] - d1 * s[szs_idx_1];
    s[pg + 2] = s[pg + 2] - d1 * s[szs_idx_1 + 1];
    s[pg + 1] = s[pg + 1] / md[nx];
    s[pg + 2] = s[pg + 2] / md[nx];
    for (k = nxm1; k >= 2; k--) {
      pg = ((k - 1) << 1) - 1;
      szs_idx_1 = k << 1;
      dxj = dx[k - 2];
      d1 = md[k - 1];
      s[pg + 1] = (s[pg + 1] - dxj * s[szs_idx_1]) / d1;
      s[pg + 2] = (s[pg + 2] - dxj * s[szs_idx_1 + 1]) / d1;
    }

    s[0] = (s[0] - d31 * s[2]) / md[0];
    s[1] = (s[1] - d31 * s[3]) / md[0];
    nxm1 = x.size(1);
    nx = (x.size(1) - 1) << 1;
    pp1_coefs.set_size(2, (x.size(1) - 1), 4);
    for (int j = 0; j <= nxm1 - 2; j++) {
      dxj = dx[j];
      szdvdf_idx_1 = (j << 1) - 1;
      d1 = dvdf[szdvdf_idx_1 + 1];
      dnnm2 = (d1 - s[szdvdf_idx_1 + 1]) / dxj;
      d1 = (s[szdvdf_idx_1 + 3] - d1) / dxj;
      pp1_coefs[szdvdf_idx_1 + 1] = (d1 - dnnm2) / dxj;
      i = nx + szdvdf_idx_1;
      pp1_coefs[i + 1] = 2.0 * dnnm2 - d1;
      szs_idx_1 = (nx << 1) + szdvdf_idx_1;
      pp1_coefs[szs_idx_1 + 1] = s[szdvdf_idx_1 + 1];
      pg = yoffset + szdvdf_idx_1;
      k = 3 * nx + szdvdf_idx_1;
      pp1_coefs[k + 1] = y[pg + 2];
      d1 = dvdf[szdvdf_idx_1 + 2];
      dnnm2 = (d1 - s[szdvdf_idx_1 + 2]) / dxj;
      d1 = (s[szdvdf_idx_1 + 4] - d1) / dxj;
      pp1_coefs[szdvdf_idx_1 + 2] = (d1 - dnnm2) / dxj;
      pp1_coefs[i + 2] = 2.0 * dnnm2 - d1;
      pp1_coefs[szs_idx_1 + 2] = s[szdvdf_idx_1 + 2];
      pp1_coefs[k + 2] = y[pg + 3];
    }

    pp_coefs.set_size(2, pp1_coefs.size(1), 4);
    szdvdf_idx_1 = pp1_coefs.size(0) * pp1_coefs.size(1) * pp1_coefs.size(2);
    for (i = 0; i < szdvdf_idx_1; i++) {
      pp_coefs[i] = pp1_coefs[i];
    }
  }

  coefs.set_size(div_s32((pp_coefs.size(1) << 1) * pp_coefs.size(2),
    pp_coefs.size(2)), pp_coefs.size(2));
  szdvdf_idx_1 = div_s32((pp_coefs.size(1) << 1) * pp_coefs.size(2),
    pp_coefs.size(2)) * pp_coefs.size(2);
  for (i = 0; i < szdvdf_idx_1; i++) {
    coefs[i] = pp_coefs[i];
  }
}

static int div_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int b_numerator;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    unsigned int b_denominator;
    if (numerator < 0) {
      b_numerator = ~static_cast<unsigned int>(numerator) + 1U;
    } else {
      b_numerator = static_cast<unsigned int>(numerator);
    }

    if (denominator < 0) {
      b_denominator = ~static_cast<unsigned int>(denominator) + 1U;
    } else {
      b_denominator = static_cast<unsigned int>(denominator);
    }

    b_numerator /= b_denominator;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -static_cast<int>(b_numerator);
    } else {
      quotient = static_cast<int>(b_numerator);
    }
  }

  return quotient;
}

// End of code generation (Spline.cpp)
