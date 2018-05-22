// cwi_encode.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "cwi_encode.h"
//#include <apps/evaluate_compression/include/pcl/apps/evaluate_compression/evaluate_compression.h>
//#include <apps/evaluate_compression/include/pcl/apps/evaluate_compression/impl/evaluate_compression.hpp>

//#include <evaluate_compression_impl.hpp>


// This is an example of an exported variable
CWI_ENCODE_API int ncwi_encode=0;

// This is an example of an exported function.
CWI_ENCODE_API int fncwi_encode(void)
{
    return 42;
}

// This is the constructor of a class that has been exported.
// see cwi_encode.h for the class definition
Ccwi_encode::Ccwi_encode()
{
    return;
}
