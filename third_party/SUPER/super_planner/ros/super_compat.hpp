#pragma once
// SUPER build compatibility for GCC 11 / C++17:
// upstream uses unqualified isnan()/isinf() without including <cmath>.
#include <cmath>
using std::isnan;
using std::isinf;
