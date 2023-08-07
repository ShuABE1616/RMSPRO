/*
Project Name    : RMSPRO
File Name       : RMSPRO.h
Creation Date   : c++
author          : Shu ABE,Hibiki SHINOHARA,Yuri HIRATA
update date     : 2023/8/7 

Copyright © <2023> RobomatchProject

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef RMSPRO_h
#define RMSPRO_h
#include <Arduino.h>

typedef enum //センサの方向指定で使用
{
	front,
	right,
	back,
	left,
} DIR;

typedef enum //センサの方向指定で使用
{
	fr,
	ri,
	ba,
	le,
} DIR2;

typedef enum
{
    d1,
    d2,
    d3,
    d4,
} number;//センサの番号指定で使用

class RMSPRO{

    public:
        RMSPRO(void);
        void init(void);

        //beginner mode
        void m(int left, int right);
        void srmo(void);
        int b(void);
        int b(number num);
        int b(DIR dir);
        int b(DIR2 dir);
        int l(void);
        int bt(void);
        int srmo(void);

        //advance mode
        void motor(int left, int right);
        void sensorMonitor(void);
        int ball(void);
        int ball(number num);
        int ball(DIR dir);
        int ball(DIR2 dir);
        int line(void);
        int btn(void);
        int sensormonitor(void);

    private:
            static void irUpdate(void);
            static void timerISR(void);
};

#endif
