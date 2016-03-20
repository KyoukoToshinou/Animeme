#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include <map>
#include <io.h>
#include <stdio.h>
#include <string>
#include <Windows.h> // Wincon.h
#include <sys/stat.h>
#include <vector>
#include <math.h>
#include <emmintrin.h>
#include <xmmintrin.h>

#ifdef DrawText
#undef DrawText
#endif

#include "math.h"
#include "language.h"
#include "file.h"
#include "utils.h"
#include "console.h"
#include "color.h"
#include "vector.h"
#include "entity.h"
#include "entmgr.h"

#include "okaeri.h"

#include "hook.h"
#include "detourxs.h"
#include "gamemanager.h"
#include "cod4.h"
#include "cod8.h"
#include "bf3.h"
#include "draw.h"
#include "game.h"