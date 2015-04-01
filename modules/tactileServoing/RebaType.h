/*
    This file is part of VTFS--Visuo-Tactile-Force-Servoing.

    VTFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    VTFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CBF.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/

#pragma once

enum MotionDirT{
	UP = 0,
	DOWN = 1,
	BACK = 2,
	FORWARD = 3,
	APPROACH = 4,
	AWAY = 5,
	LEFT = 6,
	RIGHT = 7,
	RXP = 8,
	RXN = 9,
	RYP = 10,
	RYN = 11,
	RZP = 12,
	RZN = 13,
	STOP = 14,
    NOCONTROL = 15,
    VISGUIDE = 16
};

enum RobotModeT{
    NormalMode = 0,
    PsudoGravityCompensation = 1
};

enum ForceCtrlT{
    FTCTRL = 0,
    TACCTRL = 1
};




