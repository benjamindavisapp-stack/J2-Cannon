#pragma once
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(type) for (int _atomic_once = 1; _atomic_once; _atomic_once = 0)
