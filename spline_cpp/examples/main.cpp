//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  main.cpp
//
//  Code generation for function 'main'
//


//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include files
#include <iostream>
#include <cmath>
#include "main.h"
#include "Spline.h"

// Function Declarations
static coder::array<double, 2U> argInit_1xUnbounded_real_T();
static coder::array<double, 2U> argInit_2xUnbounded_real_T();
static double argInit_real_T();
static void main_spline_cpp(Spline *instancePtr);

// Function Definitions
static coder::array<double, 2U> argInit_1xUnbounded_real_T()
{
  coder::array<double, 2U> result;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result.set_size(1, 8);

  // Loop over the array to initialize each element.
  // for (int idx1 = 0; idx1 < result.size(1); idx1++) {
  //   // Set the value of the array element.
  //   // Change this value to the value that the application requires.
  //   result[idx1] = argInit_real_T();
  // }
  result = {0, 1, 2.5, 3.6, 5, 7, 8.1, 10};

  return result;
}

static coder::array<double, 2U> argInit_2xUnbounded_real_T()
{
  coder::array<double, 2U> result;
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result.set_size(2, 8);

  // Loop over the array to initialize each element.
  // for (idx1 = 0; idx1 < result.size(1); idx1++) {
  //   // Set the value of the array element.
  //   // Change this value to the value that the application requires.
  //   result[2 * idx1] = argInit_real_T();
  // }

  // for (idx1 = 0; idx1 < result.size(1); idx1++) {
  //   // Set the value of the array element.
  //   // Change this value to the value that the application requires.
  //   result[2 * idx1 + 1] = argInit_real_T();
  // }
  result = {std::sin(0), std::cos(0), std::sin(1), std::cos(1), std::sin(2.5), std::cos(2.5), std::sin(3.6), std::cos(3.6), 
            std::sin(5), std::cos(5), std::sin(7), std::cos(7), std::sin(8.1), std::cos(8.1), std::sin(10), std::cos(10)};

  return result;
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_spline_cpp(Spline *instancePtr)
{
  coder::array<double, 2U> x;
  coder::array<double, 2U> y;
  coder::array<double, 2U> coefs;

  // Initialize function 'spline_cpp' input arguments.
  // Initialize function input argument 'x'.
  x = argInit_1xUnbounded_real_T();

  // Initialize function input argument 'y'.
  y = argInit_2xUnbounded_real_T();

  // Call the entry-point 'spline_cpp'.
  instancePtr->spline_cpp(x, y, coefs);

  for (int i=0; i<14; i++){
    for (int j=0; j<4; j++){
      std::cout<<coefs[i + j*14]<<",";
    }
    std::cout<<std::endl;
  }
}

int main(int, const char * const [])
{
  Spline *classInstance;
  classInstance = new Spline;

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_spline_cpp(classInstance);
  delete classInstance;
  return 0;
}

// End of code generation (main.cpp)
