#pragma once
#include <stdio.h>
#include <string.h>
#include "helper_3dmath.h"
#include <csignal>
#include <iostream>
#include <bitset>

//define DEBUG TO DISPLAY DEBUG INFORMATION... WARNING THIS ADDS A LOT OF TEXT TO THE CONSOLE SO IF YOU DO NOT NEED IT DO NOT TURN IT ON.
//#define DEBUG
#ifdef DEBUG
const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");
#define DEBUG_PRINT(x) std::cout << x
#define DEBUG_ERROR(x) std::cout << red << x << reset
#define DEBUG_PRINTH(x) std::cout << yellow <<"0x" << std::hex << static_cast<int>(x) << reset
#define DEBUG_PRINTB(x) std::cout << cyan << "B" << std::bitset<8>(x) << reset
#define DEBUG_PRINTW(x) std::cout << green << x << reset
#define DEBUG_PRINTR(x) std::cout << magenta << x << reset
#define DEBUG_PRINTF(x, y) std::cout << y << x
#define DEBUG_PRINTLN(x) std::cout << x << std::endl
#define DEBUG_ERRORLN(x) std::cout << red << x << reset << std::endl
#define DEBUG_PRINTLNH(x) std::cout << yellow << "0x" << std::hex << static_cast<int>(x) << reset << std::endl
#define DEBUG_PRINTLNF(x, y) std::cout << y << x << std::endl
#define DEBUG_FLUSH(x) std::cout << x << std::flush
#else
#define DEBUG_PRINT(x)
#define DEBUG_ERROR(x)
#define DEBUG_PRINTH(y)
#define DEBUG_PRINTB(x)
#define DEBUG_PRINTW(x)
#define DEBUG_PRINTR(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNH(x)
#define DEBUG_PRINTLNF(x, y)
#define DEBUG_ERRORLN
#define DEBUG_FLUSH

#endif