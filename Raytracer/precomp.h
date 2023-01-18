#pragma once

extern "C" {
	#pragma warning(push)
	#pragma warning(disable: 26819)
	#pragma warning(disable: 26812)
    #include "SDL.h"
	#pragma warning(pop)
}

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "robustwin32io.h"
#include "microui.h"

#include <stdint.h>
#include <stddef.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <functional>

#pragma warning(disable: 26812) // "Prefer 'enum class' over 'enum'": I disagree, go away.
#pragma warning(disable: 4201)  // "nonstandard extension used: nameless struct/union": I do this all the time, and I know it's a non-standard extension.
#pragma warning(disable: 4723)  // "potential divide by 0": this fires, but points you completely to the wrong code, and is impossible to understand what it's referring to

// TODO: Not sure about these yet, but for now in development they are very annoying. I have historically had bugs that this would catch, but is it worth it?
#pragma warning(disable: 4100) // "unreferenced formal parameter"
#pragma warning(disable: 4101) // "unreferenced local variable"
#pragma warning(disable: 4189) // "local variable is initialized but not referenced"
#pragma warning(disable: 4505) // "unreferenced local variable has been removed"

#include "..\MathLib\my_math.h"
#include "..\MathLib\simd_math_4x.h"
using namespace math;
