#!/usr/bin/env python3
from typing import Dict

color = tuple[int,int,int]

class Colors(Dict):
    red = (255,0,0)
    green = (0,255,0)
    blue = (0,0,255)
    purple = (255,0,255)
    cyan = (0,255,255)
    yellow = (255,255,0)
    black = (0,0,0)
    white = (255,255,255)
    gray = (100,100,100)