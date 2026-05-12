#include <iostream>
#include <string>
#include <OLED_library.h>
#include <format>

int main()
{
    std::cout << std::format("Current OLED version: {}\n", OLED_Version());

    return 0;
}
