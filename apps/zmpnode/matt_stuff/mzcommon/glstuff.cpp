/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "glstuff.h"
#include <iostream>

namespace glstuff {

size_t pwr2(size_t x) {

  size_t i=0;

  if (!x) { return 0; }

  x = x-1;
  while (x) {
    x = (x >> 1);
    ++i;
  }

  return (1 << i);

}


void check_opengl_errors(const char* context) {
  GLenum error = glGetError();
  if (!context) { context = "error"; }
  if (error) {
    std::cerr << context << ": " << gluErrorString(error) << "\n";
  }
}

}
