import math
import time
import numpy as np
from math import cos,sin,pi
from utils.misc import TimerStat
from OpenGL.GL import *
from OpenGL.GLUT import *
from PIL.Image import open
import xml.dom.minidom
EGO_LENGTH = 4.8
EGO_WIDTH = 2.0
STATE_OTHER_LENGTH = EGO_LENGTH
STATE_OTHER_WIDTH = EGO_WIDTH

def rotate_coordination(orig_x, orig_y, orig_d, coordi_rotate_d):
    """
    :param orig_x: original x
    :param orig_y: original y
    :param orig_d: original degree
    :param coordi_rotate_d: coordination rotation d, positive if anti-clockwise, unit: deg
    :return:
    transformed_x, transformed_y, transformed_d(range:(-180 deg, 180 deg])
    """

    coordi_rotate_d_in_rad = coordi_rotate_d * math.pi / 180
    transformed_x = orig_x * math.cos(coordi_rotate_d_in_rad) + orig_y * math.sin(coordi_rotate_d_in_rad)
    transformed_y = -orig_x * math.sin(coordi_rotate_d_in_rad) + orig_y * math.cos(coordi_rotate_d_in_rad)
    transformed_d = orig_d - coordi_rotate_d
    if transformed_d > 180:
        while transformed_d > 180:
            transformed_d = transformed_d - 360
    elif transformed_d <= -180:
        while transformed_d <= -180:
            transformed_d = transformed_d + 360
    else:
        transformed_d = transformed_d
    return transformed_x, transformed_y, transformed_d

class Render():
    def __init__(self, shared_list, path_index, lock, task, model_only_test=False):
        self.shared_list = shared_list
        self.lock = lock
        self.task = task
        self.model_only_test = model_only_test
        self.path_index = path_index
        self.step_old = -1
        self.acc_timer = TimerStat()
        self._load_xml()
        self.red_img = self._read_png('utils/Rendering/red.png')
        self.green_img = self._read_png('utils/Rendering/green.png')
        self.GL_TEXTURE_RED = glGenTextures(1)
        self.GL_TEXTURE_GREEN = glGenTextures(1)
        left_construct_traj = np.load('./map/left_ref.npy')
        straight_construct_traj = np.load('./map/straight_ref.npy')
        right_construct_traj = np.load('./map/right_ref.npy')
        self.ref_path_all = {'left': left_construct_traj, 'straight': straight_construct_traj,
                             'right': right_construct_traj}


    def run(self):
        self._opengl_start()

    def _read_png(self, path):
        im = open(path)
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBA", 0, -1)
        return ix, iy, image

    def _opengl_start(self):
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH)
        glutInitContextProfile(GLUT_CORE_PROFILE)
        glutInitWindowSize(1000, 1000)
        glutInitWindowPosition(460, 0)
        glutCreateWindow('Crossroad')
        glutDisplayFunc(self.render)
        # glutIdleFunc(self.render)
        glutTimerFunc(20,self.render, 0)
        glutMainLoop()

    def _load_xml(self, path="./utils/sumo_files/a.net.xml"):
        dom_obj = xml.dom.minidom.parse(path)
        element_obj = dom_obj.documentElement
        self.sub_element_edge = element_obj.getElementsByTagName("edge")
        self.sub_element_junction = element_obj.getElementsByTagName("junction")
        self.sub_element_tlLogic = element_obj.getElementsByTagName("tlLogic")

    def _draw_map(self, scale):
        sub_element_edge = self.sub_element_edge
        sub_element_junction = self.sub_element_junction

        def bit_pattern(*args):
            args = list(args[:16])
            args = args + [0] * (16 - len(args))
            base = 0
            for arg in args:
                base = base << 1
                base += bool(arg)
            return base

        for i in range(len(sub_element_edge)):
            if sub_element_edge[i].getAttribute('function') != 'internal':
                sub_element_lane = sub_element_edge[i].getElementsByTagName("lane")
                for j in range(len(sub_element_lane)):
                    shape = sub_element_lane[j].getAttribute("shape")
                    type = str(sub_element_lane[j].getAttribute("allow"))
                    try:
                        width = float(sub_element_lane[j].getAttribute("width"))
                    except:
                        width = 3.5
                    shape_list = shape.split(" ")
                    for k in range(len(shape_list) - 1):
                        shape_point_1 = shape_list[k].split(",")
                        shape_point_2 = shape_list[k + 1].split(",")
                        shape_point_1[0] = float(shape_point_1[0])
                        shape_point_1[1] = float(shape_point_1[1])
                        shape_point_2[0] = float(shape_point_2[0])
                        shape_point_2[1] = float(shape_point_2[1])
                        # 道路顶点生成
                        dx1 = shape_point_2[0] - shape_point_1[0]
                        dy1 = shape_point_2[1] - shape_point_1[1]
                        v1 = np.array([-dy1, dx1])
                        absdx = abs(shape_point_1[0] - shape_point_2[0])
                        absdy = abs(shape_point_1[1] - shape_point_2[1])
                        if math.sqrt(absdx * absdx + absdy * absdy) > 0:
                            v3 = v1 / math.sqrt(absdx * absdx + absdy * absdy) * width * 0.5
                            [x1, y1] = ([shape_point_1[0], shape_point_1[1]] + v3) / scale
                            [x2, y2] = ([shape_point_1[0], shape_point_1[1]] - v3) / scale
                            [x4, y4] = ([shape_point_2[0], shape_point_2[1]] + v3) / scale
                            [x3, y3] = ([shape_point_2[0], shape_point_2[1]] - v3) / scale  # 0.0176 * v3
                            glBegin(GL_POLYGON)  # 开始绘制单车道
                            if type == 'pedestrian':
                                glColor3f(0.663, 0.663, 0.663)
                            elif type == 'bicycle':
                                glColor3f(0.455, 0.721, 0.926)
                            else:
                                glColor3f(0.0, 0.0, 0.0)
                            glVertex2f(x1, y1)
                            glVertex2f(x2, y2)
                            glVertex2f(x3, y3)
                            glVertex2f(x4, y4)
                            glEnd()

                            glLineWidth(1.0)
                            glLineStipple(3, bit_pattern(
                                0, 0, 0, 0,
                                0, 0, 0, 0,
                                1, 1, 1, 1,
                                1, 1, 1, 1,
                            ))
                            glEnable(GL_LINE_STIPPLE)
                            glBegin(GL_LINES)
                            glColor3f(1.0, 1.0, 1.0)
                            glVertex2f(x2, y2)
                            glVertex2f(x3, y3)
                            glVertex2f(x1, y1)
                            glVertex2f(x4, y4)
                            glEnd()
                            glDisable(GL_LINE_STIPPLE)
                            glLineWidth(10.0)
                            glBegin(GL_LINES)
                            glColor3f(1.0, 1.0, 1.0)
                            glVertex2f(x3, y3)
                            glVertex2f(x4, y4)
                            glEnd()
                            glLineWidth(2.0)
                            glBegin(GL_LINES)
                            glColor3f(0.8275, 0.8275, 0.8275)
                            if j == sub_element_lane.length - 1:
                                glVertex2f(x1, y1)
                                glVertex2f(x4, y4)
                            if j == 0:
                                glVertex2f(x2, y2)
                                glVertex2f(x3, y3)
                            glEnd()

        for i in range(len(sub_element_junction)):
            shape = sub_element_junction[i].getAttribute("shape")
            shape_list = shape.split(" ")

            glLineWidth(1)
            glBegin(GL_POLYGON)
            glColor3f(0.0, 0.0, 0.0)
            for k in range(len(shape_list)):
                shape_point = shape_list[k].split(",")
                if shape_point[0] != '':
                    glVertex2f((float(shape_point[0]) / scale) * 1, (float(shape_point[1]) / scale) * 1)
            glEnd()
            glutTimerFunc(20,self.render,0)


    def _draw_zebra(self, loc, width, length, scale, shape, single_height=0.8):
        glLineWidth(1)
        glColor3f(0.8275, 0.8275, 0.8275)
        if shape == 'vertical':
            for i in range(int(length/single_height)):
                glBegin(GL_POLYGON)
                x1, y1 = (loc - width / 2) / scale, (2 * i * single_height) / scale
                x2, y2 = (loc + width / 2) / scale, (2 * i * single_height) / scale
                x3, y3 = (loc + width / 2) / scale, ((2 * i + 1) * single_height) / scale
                x4, y4 = (loc - width / 2) / scale, ((2 * i + 1) * single_height) / scale
                glVertex2f(x1, y1)
                glVertex2f(x2, y2)
                glVertex2f(x3, y3)
                glVertex2f(x4, y4)
                glEnd()
                glBegin(GL_POLYGON)
                x1, y1 = (loc - width / 2) / scale, -((2 * i + 2) * single_height) / scale
                x2, y2 = (loc + width / 2) / scale, -((2 * i + 2) * single_height) / scale
                x3, y3 = (loc + width / 2) / scale, -((2 * i + 1) * single_height) / scale
                x4, y4 = (loc - width / 2) / scale, -((2 * i + 1) * single_height) / scale
                glVertex2f(x1, y1)
                glVertex2f(x2, y2)
                glVertex2f(x3, y3)
                glVertex2f(x4, y4)
                glEnd()
        elif shape == 'horizontal':
            for i in range(int(length/single_height)):
                glBegin(GL_POLYGON)
                y1, x1 = (loc - width / 2) / scale, (2 * i * single_height) / scale
                y2, x2 = (loc + width / 2) / scale, (2 * i * single_height) / scale
                y3, x3 = (loc + width / 2) / scale, ((2 * i + 1) * single_height) / scale
                y4, x4 = (loc - width / 2) / scale, ((2 * i + 1) * single_height) / scale
                glVertex2f(x1, y1)
                glVertex2f(x2, y2)
                glVertex2f(x3, y3)
                glVertex2f(x4, y4)
                glEnd()
                glBegin(GL_POLYGON)
                y1, x1 = (loc - width / 2) / scale, -((2 * i + 2) * single_height) / scale
                y2, x2 = (loc + width / 2) / scale, -((2 * i + 2) * single_height) / scale
                y3, x3 = (loc + width / 2) / scale, -((2 * i + 1) * single_height) / scale
                y4, x4 = (loc - width / 2) / scale, -((2 * i + 1) * single_height) / scale
                glVertex2f(x1, y1)
                glVertex2f(x2, y2)
                glVertex2f(x3, y3)
                glVertex2f(x4, y4)
                glEnd()

    def _plot_reference(self, task, highlight_index, scale):
        glPointSize(2.0)
        glBegin(GL_POINTS)
        for i in range(self.ref_path_all[task].shape[0]):
            if i != highlight_index:
                glColor3f(0.0, 0.4, 0.0)
                for j in range(self.ref_path_all[task].shape[2]):
                    x = self.ref_path_all[task][i][0][j] / scale
                    y =  self.ref_path_all[task][i][1][j] / scale
                    glVertex2f(x, y)
        for i in range(self.ref_path_all[task].shape[0]):
            if i == highlight_index:
                glColor3f(0.486, 0.99, 0.0)
                for j in range(self.ref_path_all[task].shape[2]):
                    x = self.ref_path_all[task][i][0][j] / scale
                    y =  self.ref_path_all[task][i][1][j] / scale
                    glVertex2f(x, y)
        glEnd()





    def render(self, real_x=0, real_y=0, scale=60, **kwargs):
        time_st = time.time()
        LOC_X = -real_x / scale
        LOC_Y = -real_y / scale
        glClearColor(0.1333, 0.545, 0.1333, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(LOC_X, LOC_Y, 0)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_POLYGON_SMOOTH)
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

        # draw map
        self._draw_map(scale)
        self._draw_zebra(-17, 6, 7, scale, 'vertical')
        self._draw_zebra(17, 6, 7, scale, 'vertical')
        self._draw_zebra(26, 6, 4, scale, 'horizontal')
        self._draw_zebra(-22, 6, 4, scale, 'horizontal')

        # draw ref
        self._plot_reference(self.task, self.path_index, scale)

        # draw vehicles
        def draw_rotate_rec(x, y, a, l, w, scale, color='o'):
            RU_x, RU_y, _ = rotate_coordination(l / 2, w / 2, 0, -a)
            RD_x, RD_y, _ = rotate_coordination(l / 2, -w / 2, 0, -a)
            LU_x, LU_y, _ = rotate_coordination(-l / 2, w / 2, 0, -a)
            LD_x, LD_y, _ = rotate_coordination(-l / 2, -w / 2, 0, -a)
            glBegin(GL_POLYGON)
            if color == 'o':
                glColor3f(1.0, 0.647, 0.0)
            elif color == 'y':
                glColor3f(1.0, 1.0, 0.878)
            glVertex2f((RU_x + x) / scale, (RU_y + y) / scale)
            glVertex2f((RD_x + x) / scale, (RD_y + y) / scale)
            glVertex2f((LD_x + x) / scale, (LD_y + y) / scale)
            glVertex2f((LU_x + x) / scale, (LU_y + y) / scale)
            glEnd()

        def plot_phi_line(x, y, phi, color, scale):
            line_length = 5
            x_forw, y_forw = x + line_length * cos(phi * pi / 180.), \
                             y + line_length * sin(phi * pi / 180.)
            glLineWidth(1.0)
            glBegin(GL_LINES)
            if color == 'o':
                glColor3f(1.0, 0.647, 0.0)
            elif color == 'y':
                glColor3f(1.0, 1.0, 0.878)
            glVertex2f(x / scale,y / scale)
            glVertex2f(x_forw / scale,y_forw / scale)
            glEnd()


        # ego vehicle
        ego_x = 0
        ego_y = 0
        ego_phi = 0
        draw_rotate_rec(ego_x, ego_y, ego_phi, EGO_LENGTH, EGO_WIDTH, scale, color='o')
        plot_phi_line(ego_x,ego_y,ego_phi,'o',scale)

        state_other = self.shared_list[4].copy()
        # plot cars
        for veh in state_other:
            veh_x = veh['x']
            veh_y = veh['y']
            veh_phi = veh['phi']
            veh_l = STATE_OTHER_LENGTH
            veh_w = STATE_OTHER_WIDTH
            plot_phi_line(veh_x, veh_y, veh_phi, 'y', scale)
            draw_rotate_rec(veh_x, veh_y, veh_phi, veh_l, veh_w, scale, color='y')

        v_light = 0 # todo
        if v_light == 0:
            self._texture_light(self.green_img, (-8, 20), 'U', scale)
            self._texture_light(self.green_img, (0, -18), 'D', scale)
            self._texture_light(self.red_img, (-14, -12), 'L', scale)
            self._texture_light(self.red_img, (11, 4), 'R', scale)
        elif v_light != 0:
            self._texture_light(self.red_img, (-8, 20), 'U', scale)
            self._texture_light(self.red_img, (0, -18), 'D', scale)
            self._texture_light(self.green_img, (-14, -12), 'L', scale)
            self._texture_light(self.green_img, (11, 4), 'R', scale)

        glutSwapBuffers()

        glDisable(GL_BLEND)
        glDisable(GL_LINE_SMOOTH)
        glDisable(GL_POLYGON_SMOOTH)
        print(time.time()-time_st)


    def _texture_light(self, img, loc, edge, scale, size=(8, 3)):
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.GL_TEXTURE_RED)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)

        glTexImage2D(GL_TEXTURE_2D, 0, 3, img[0], img[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, img[2])

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        if edge == 'U' or edge=='D':
            x1, y1 = loc[0] / scale, loc[1] / scale
            x2, y2 = (loc[0] + size[0]) / scale, loc[1] / scale
            x3, y3 = loc[0] / scale, (loc[1] + size[1]) / scale
            x4, y4 = (loc[0] + size[0]) / scale, (loc[1] + size[1]) / scale
        elif edge == 'L' or edge=='R':
            x1, y1 = loc[0] / scale, loc[1] / scale
            x2, y2 = (loc[0] + size[1]) / scale, loc[1] / scale
            x3, y3 = loc[0] / scale, (loc[1] + size[0]) / scale
            x4, y4 = (loc[0] + size[1]) / scale, (loc[1] + size[0]) / scale
        glEnable(GL_TEXTURE_2D)
        glBegin(GL_QUADS)
        if edge == 'U':
            glTexCoord2f(0.0, 0.0)
            glVertex3f(x4, y4, 0.0)
            glTexCoord2f(1.0, 0.0)
            glVertex3f(x3, y3, 0.0)
            glTexCoord2f(1.0, 1.0)
            glVertex3f(x1, y1, 0.0)
            glTexCoord2f(0.0, 1.0)
            glVertex3f(x2, y2, 0.0)
        elif edge == 'D':
            glTexCoord2f(0.0, 0.0)
            glVertex3f(x1, y1, 0.0)
            glTexCoord2f(1.0, 0.0)
            glVertex3f(x2, y2, 0.0)
            glTexCoord2f(1.0, 1.0)
            glVertex3f(x4, y4, 0.0)
            glTexCoord2f(0.0, 1.0)
            glVertex3f(x3, y3, 0.0)
        elif edge == 'L':
            glTexCoord2f(0.0, 0.0)
            glVertex3f(x3, y3, 0.0)
            glTexCoord2f(1.0, 0.0)
            glVertex3f(x1, y1, 0.0)
            glTexCoord2f(1.0, 1.0)
            glVertex3f(x2, y2, 0.0)
            glTexCoord2f(0.0, 1.0)
            glVertex3f(x4, y4, 0.0)
        elif edge == 'R':
            glTexCoord2f(0.0, 0.0)
            glVertex3f(x2, y2, 0.0)
            glTexCoord2f(1.0, 0.0)
            glVertex3f(x4, y4, 0.0)
            glTexCoord2f(1.0, 1.0)
            glVertex3f(x3, y3, 0.0)
            glTexCoord2f(0.0, 1.0)
            glVertex3f(x1, y1, 0.0)

        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_ALPHA_TEST)
        glDisable(GL_BLEND)




if __name__ == '__main__':
    path_index = 0
    share_list = [{'x':0.0, 'y':10.0,'phi':135.0}]
    render = Render(share_list, path_index, None, 'right')
    render.run()
