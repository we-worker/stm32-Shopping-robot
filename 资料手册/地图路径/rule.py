#!/usr/bin/python
# -*- coding: utf-8 -*-
import turtle
'''
@author:   hamlin
@homepage: http://www.hamlinzheng.com
@github:   http://github.com/hamlinzheng
LastEditTime: 2022-04-23 14:24:54
'''
'''
@author:   luheqiu
@homepage: http://www.luheqiu.com
@github:   http://gitee.com/luheqiu
LastEditTime : 2021-12-04 12:14:43
'''

''' 坐标系定义
    y
    |
    |
    |
    |
    —— —— —— ——> x
'''

MAP_CONFIG = [
    [1,0,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,1,1,1,0,0,0,0],
    [1,0,0,0,1,1,1,0,0,0,0],
    [1,0,0,0,1,1,1,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,0,0,0,0],
]

# 地图一
# PRESET_Path = [
#     ['Y', 8 , 0],
#     ['X', -5 , 0],
#     ['Y', -6 , 0],
#     ['X', -1 , 0],
#     ['Y', 6 , 0],
#     ['X', 2 , 0],
#     ['Y', 1 , 0],
#     ['Y', -1 , 0],
#     ['X', 1 , 0],
#     ['Y', 2 , 0],
#     ['X', 3 , 0],
#     ['Y',-6 , 0],
#     ['X',1 , 0],
#     ['Y',1 , 0],
#     ['X',-1 , 0],
#     ['Y',-4 , 0],
# ]

# 地图二
PRESET_Path = [
    ['Y',  3 , 0],
    ['X', -2 , 0],
    ['Y', -1 , 0],
    ['Y',  1 , -1], #1
    ['X', -1 , 0],
    ['Y', -1 , 0],
    ['Y',  1 , -1], #2
    ['X', -1 , 0],
    ['Y', -1 , 0],
    ['Y',  1 , -1], #3
    ['X', -2 , 0],
    ['Y', -1 , 0],
    ['X',  1 , 0],
    ['Y',  1 , 0],
    ['X', -1 , 0],
    ['Y', -1 , 0],
    ['X',  1 , 0],
    ['Y',  3 , 0],
    ['X', -1 , 0],
    ['X',  1 , -1],
    ['Y',  3 , 0],
    ['X',  6 , 0],
    ['Y', -7 , 0],
    ['X', -1 , 0]
]

class Space(object):
    def __init__(self):
        self.grid_wide = 72             # 格子宽
        self.line_wide = 12             # 线宽
        self.grid_num_x = len(MAP_CONFIG[0])      # x轴格子数
        self.grid_num_y = len(MAP_CONFIG)         # y轴格子数
        self.start_x = 2.5
        self.start_y = -5.5
        self.start_wide = 72
        self._config()
        
    def _config(self):
        screen = turtle.Screen()
        screen.setup(self.grid_wide*self.grid_num_y,self.grid_wide*self.grid_num_x)
        turtle.hideturtle()
        turtle.speed(30)     # max speed
        turtle.color('black')
        turtle.up()
        
    
    def build(self):
        init_x = - (0.5 * self.grid_num_x * self.grid_wide - 0.25 * self.line_wide)   # 重定义x零点
        init_y = - (0.5 * self.grid_num_y * self.grid_wide - 0.25 * self.line_wide)   # 重定义y零点
        init_wide = self.grid_wide - 0.5 * self.line_wide   # 重定义格子宽度
        for x in range(self.grid_num_x):
            for y in range(self.grid_num_y):
                if MAP_CONFIG[self.grid_num_y-1-y][x] == 1:
                    turtle.color("#E0E0E0")
                elif  MAP_CONFIG[self.grid_num_y-1-y][x] == 0:
                    turtle.color('black')
                elif  MAP_CONFIG[self.grid_num_y-1-y][x] == 2:
                    turtle.color('red')
                self.draw_square_solid(init_x + x * self.grid_wide,init_y + y * self.grid_wide, init_wide)
        self.draw_start_area()

    def path(self):
        turtle.showturtle()
        turtle.setpos(self.start_x * self.grid_wide, self.start_y * self.grid_wide)
        turtle.color('red')
        turtle.pensize(3)
        turtle.down()
        turtle.speed(1)     # speed :1,3,6,9,10, 0
        # turtle.setheading(90)
        for i in range(len(PRESET_Path)):
            if i > 3: turtle.color('red')
            if PRESET_Path[i][0] == 'X':
                if PRESET_Path[i][1] > 0 :
                    if PRESET_Path[i][2] == 0:
                        turtle.setheading(0)
                        turtle.forward(PRESET_Path[i][1] * self.grid_wide)
                    else:
                        turtle.setheading(180)
                        turtle.forward( - PRESET_Path[i][1] * self.grid_wide)
                else:
                    if PRESET_Path[i][2] == 0:
                        turtle.setheading(180)
                        turtle.forward( - PRESET_Path[i][1] * self.grid_wide)
                    else:
                        turtle.setheading(0)
                        turtle.forward(  PRESET_Path[i][1] * self.grid_wide)
                
            elif PRESET_Path[i][0] == 'Y':
                if PRESET_Path[i][1] > 0 :
                    if PRESET_Path[i][2] == 0:
                        turtle.setheading(90)
                        turtle.forward(PRESET_Path[i][1] * self.grid_wide)
                    else:
                        turtle.setheading(270)
                        turtle.forward( - PRESET_Path[i][1] * self.grid_wide) 
                else:
                    if PRESET_Path[i][2] == 0:
                        turtle.setheading(270)
                        turtle.forward( - PRESET_Path[i][1] * self.grid_wide)
                    else:
                        turtle.setheading(90)
                        turtle.forward( PRESET_Path[i][1] * self.grid_wide)
   
    # description: draw a hollow square
    def draw_square_hollow(self, posx, posy, wide):
        turtle.setpos(posx, posy)
        turtle.setheading(0)
        turtle.down()
        for i in range(4):
            turtle.forward(wide)
            turtle.left(90)
        turtle.up()
    # description: draw a solid square
    def draw_square_solid(self, posx, posy, wide):
        turtle.begin_fill()
        self.draw_square_hollow(posx, posy, wide)
        turtle.end_fill()
    # description: draw a start area
    def draw_start_area(self):
        turtle.up()
        turtle.color('red')
        self.draw_square_solid(self.start_x * self.grid_wide - 0.5 * self.grid_wide , self.start_y * self.grid_wide, self.start_wide)
        

if __name__ == "__main__":
    space = Space()
    
    space.build()
    space.path()

    turtle.exitonclick()        # 阻塞