import math
import time
from math import cos, sin, pi

import matplotlib.pyplot as plt
import numpy as np

from utils.misc import TimerStat
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL.Image import open
import cv2


class Render():
    def __init__(self, shared_list, lock, task, model_only_test=False):
        self.shared_list = shared_list
        self.lock = lock
        self.task = task
        self.model_only_test = model_only_test
        self.step_old = -1
        self.acc_timer = TimerStat()
        self._opengl_init()

    def _opengl_init(self):
        glutInit()  # 1. 初始化glut库
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH)

        glutInitContextProfile(GLUT_CORE_PROFILE)
        glutInitWindowSize(600, 600)
        glutInitWindowPosition(0, 0)
        glutCreateWindow('Quidam Of OpenGL')  # 2. 创建glut窗口
        glutDisplayFunc(self.test_texture)  # 3. 注册回调函数draw()
        # glutTimerFunc(500, self.test_draw_2, None)
        glutMainLoop()  # 4. 进入glut主循环

    def test_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # ---------------------------------------------------------------
        glBegin(GL_LINES)  # 开始绘制线段（世界坐标系）

        # 以红色绘制x轴
        glColor4f(1.0, 0.0, 0.0, 1.0)  # 设置当前颜色为红色不透明
        glVertex3f(-0.8, 0.0, 0.0)  # 设置x轴顶点（x轴负方向）
        glVertex3f(0.8, 0.0, 0.0)  # 设置x轴顶点（x轴正方向）

        # 以绿色绘制y轴
        glColor4f(0.0, 1.0, 0.0, 1.0)  # 设置当前颜色为绿色不透明
        glVertex3f(0.0, -0.8, 0.0)  # 设置y轴顶点（y轴负方向）
        glVertex3f(0.0, 0.8, 0.0)  # 设置y轴顶点（y轴正方向）

        # 以蓝色绘制z轴
        glColor4f(0.0, 0.0, 1.0, 1.0)  # 设置当前颜色为蓝色不透明
        glVertex3f(0.0, 0.0, -0.8)  # 设置z轴顶点（z轴负方向）
        glVertex3f(0.0, 0.0, 0.8)  # 设置z轴顶点（z轴正方向）

        glEnd()  # 结束绘制线段

        # ---------------------------------------------------------------
        glBegin(GL_TRIANGLES)  # 开始绘制三角形（z轴负半区）

        glColor4f(1.0, 0.0, 0.0, 0.5)  # 设置当前颜色为红色不透明
        glVertex3f(-0.5, -0.366, -0.5)  # 设置三角形顶点
        glColor4f(0.0, 1.0, 0.0, 0.5)  # 设置当前颜色为绿色不透明
        glVertex3f(0.5, -0.366, -0.5)  # 设置三角形顶点
        glColor4f(0.0, 0.0, 1.0, 0.5)  # 设置当前颜色为蓝色不透明
        glVertex3f(0.0, 0.5, -0.5)  # 设置三角形顶点

        glEnd()  # 结束绘制三角形

        # ---------------------------------------------------------------
        glFlush()  # 清空缓冲区，将指令送往硬件立即执行

    def test_texture(self):
        # glShadeModel(GL_SMOOTH)  # 平滑着色
        glEnable(GL_DEPTH_TEST)  # 深度测试
        # glDepthMask(GL_FALSE)
        # glEnable(GL_CULL_FACE)  # 只渲染某一面
        # glFrontFace(GL_CCW)  # 逆时针正面
        glEnable(GL_ALPHA_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)  # 启用2D纹理映射
        glAlphaFunc(GL_GREATER, 0.9)
        # 载入纹理图像：
        # ReadImage()
        im = open('./utils/Rendering/100.png')
        # im = im.convert('RGBA')
        # r, g, b, alpha = im.split()
        # alpha = alpha.point(lambda i: i > 0 and 80)
        # im.putalpha(alpha)
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBA", 0, -1)

        # 生成纹理对象：
        ID = glGenTextures(1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glBindTexture(GL_TEXTURE_2D, ID)  # 绑定纹理：

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)  # 支持4字节对齐

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)  # S方向上贴图
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)  # T方向上贴图
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)  # 放大纹理过滤方式
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)  # 缩小纹理过滤方式

        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)  # 载入纹理：

        glMatrixMode(GL_MODELVIEW)  # 选择模型观察矩阵
        glLoadIdentity()  # 重置模型观察矩阵
        glMatrixMode(GL_PROJECTION)  # 选择投影矩阵
        glLoadIdentity()

        glEnable(GL_TEXTURE_2D)  # 启用2D纹理映射
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0)
        glVertex3f(-0.5, -0.5, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex3f(0.5, -0.5, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex3f(0.5, 0.5, 0.0)
        glTexCoord2f(0.0, 1.0)
        glVertex3f(-0.5, 0.5, 0.0)
        glEnd()
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0)
        glVertex3f(-1.0, 0.0, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex3f(0.0, -1.0, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex3f(1.0, 0.0, 0.0)
        glTexCoord2f(0.0, 1.0)
        glVertex3f(0.0, 1.0, 0.0)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_ALPHA_TEST)
        glDisable(GL_BLEND)

        glutSwapBuffers()


if __name__ == '__main__':
    render = Render(None, None, None)
