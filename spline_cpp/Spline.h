//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Spline.h
//
//  Code generation for function 'Spline'
//


#ifndef SPLINE_H
#define SPLINE_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "spline_cpp_types.h"

// Type Definitions
class Spline
{
 public:
  Spline();
  ~Spline();
  void spline_cpp(const coder::array<double, 2U> &x, const coder::array<double,
                  2U> &y, coder::array<double, 2U> &coefs);
};

#endif

// End of code generation (Spline.h)
