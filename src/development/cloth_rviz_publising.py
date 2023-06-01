from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import sys, os, traceback
if sys.platform == 'win32' or sys.platform == 'win64':
    os.environ['SDL_VIDEO_CENTERED'] = '1'
from math import *
pygame.display.init()
pygame.font.init()
import numpy as np
import timeit

import rospy

import visualization_msgs.msg # for Markers
import geometry_msgs.msg

rospy.init_node('cloth_sim_node', anonymous=True)    
topic_name = "cloth_points"
pub = rospy.Publisher(topic_name, visualization_msgs.msg.Marker, queue_size=1)


def publish_rviz_points(points):
    m = visualization_msgs.msg.Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    
    m.type = visualization_msgs.msg.Marker.POINTS
    m.id = 0
    m.action = m.ADD

    m.pose.orientation.w = 1

    m.points = points
    
    m.scale.x = 0.01
    m.scale.y = 0.01
    m.scale.z = 0.01

    m.color.a = 1.
    m.color.r = 1.
    m.color.g = 0.5
    m.color.b = 0.

    pub.publish(m)

def publish_rviz_lines(points):
    m = visualization_msgs.msg.Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    
    m.type = visualization_msgs.msg.Marker.LINE_LIST
    m.id = 1
    m.action = m.ADD

    m.pose.orientation.w = 1

    m.points = points
    
    # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    m.scale.x = 0.005

    m.color.a = 1.
    m.color.r = 0.
    m.color.g = 1.
    m.color.b = 0.

    pub.publish(m)

screen_size = [800,600]
multisample = 16
#icon = pygame.Surface((1,1)); icon.set_alpha(0); pygame.display.set_icon(icon)
pygame.display.set_caption("Cloth Demo")
if multisample:
    pygame.display.gl_set_attribute(GL_MULTISAMPLEBUFFERS,1)
    pygame.display.gl_set_attribute(GL_MULTISAMPLESAMPLES,multisample)
pygame.display.set_mode(screen_size,OPENGL|DOUBLEBUF)

#glEnable(GL_BLEND)
#glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)

#glEnable(GL_TEXTURE_2D)
#glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE)
#glTexEnvi(GL_POINT_SPRITE,GL_COORD_REPLACE,GL_TRUE)

glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST)
glEnable(GL_DEPTH_TEST)

def subtract(vec1,vec2):
    return [vec1[i]-vec2[i] for i in [0,1,2]]

def get_length(vec):
    return sum([vec[i]*vec[i] for i in [0,1,2]])**0.5

class Particle(object):
    def __init__(self,pos):
        self.pos = pos
        self.last_pos = list(self.pos)
        self.accel = [0.0,0.0,0.0]
        
        self.constrained = False
        
    def move(self,dt):
        #Don't move constrained particles
        if self.constrained: return
        #Move
        for i in [0,1,2]:
            #Verlet integration
            temp = 2*self.pos[i] - self.last_pos[i] + self.accel[i]*dt*dt
            self.last_pos[i] = self.pos[i]
            self.pos[i] = temp
            

class Edge(object):
    def __init__(self, p1,p2, tolerance=0.01):
        self.p1 = p1
        self.p2 = p2
        self.tolerance = tolerance
        
        self.rest_length = get_length(subtract(self.p2.pos,self.p1.pos))
        self.lower_length = self.rest_length*(1.0-self.tolerance)
        self.upper_length = self.rest_length*(1.0+self.tolerance)
        
    def constrain(self):
        vec = [self.p2.pos[i]-self.p1.pos[i] for i in [0,1,2]]
        length = get_length(vec)

        if   length > self.upper_length:
            target_length = self.upper_length
            strength = 1.0
        elif length < self.lower_length:
            target_length = self.lower_length
            strength = 1.0
        elif length > self.rest_length:
            target_length = self.rest_length
            strength = (length - self.rest_length) / ( self.tolerance*self.rest_length)
            
        elif length < self.rest_length:
            target_length = self.rest_length
            strength = (length - self.rest_length) / (-self.tolerance*self.rest_length)
            
        else:
            return
        movement_for_each = 1.0*strength * (length - target_length) / 2.0

        for i in [0,1,2]:
            if not self.p1.constrained: self.p1.pos[i] += movement_for_each*vec[i]
            if not self.p2.constrained: self.p2.pos[i] -= movement_for_each*vec[i]
                        
class ClothCPU(object):
    def __init__(self, res):
        self.res = res

        corners = [
            [-1,-1],        [ 1,-1],

            [-1, 1],        [ 1, 1]
        ]
        edges = [
                    [ 0,-1],
            [-1, 0],        [ 1, 0],
                    [ 0, 1]
        ]
        self.rels = edges + corners
        print("self.rels: " + str(self.rels))
        self.reset()

    def reset(self):
        self.particles = []
        for z in range(self.res):
            row = []
            for x in range(self.res):
                row.append(Particle([
                    float(x)/float(self.res-1),
                    1.0,
                    float(z)/float(self.res-1)
                ]))
            self.particles.append(row)
        self.particles[         0][         0].constrained = True
        self.particles[self.res-1][         0].constrained = True
        self.particles[         0][self.res-1].constrained = True
        self.particles[self.res-1][self.res-1].constrained = True

        # # Makes an edge of the cloth completely constrained
        # for each in range(self.res):
        #     self.particles[each][self.res-1].constrained = True

        # # Makes an edge of the cloth completely constrained
        # for each in range(self.res):
        #     self.particles[each][0].constrained = True

        self.edges = []
        for z1 in range(self.res):
            for x1 in range(self.res):
                p1 = self.particles[z1][x1]
                for rel in self.rels:
                    x2 = x1 + rel[0]
                    z2 = z1 + rel[1]
                    if x2 < 0 or x2 >= self.res: continue
                    if z2 < 0 or z2 >= self.res: continue
                    p2 = self.particles[z2][x2]

                    found = False
                    for edge in self.edges: # THIS SEARCH FOR EDGE EXISTANCE IS VERY INEFFICIENT!
                        if edge.p1 == p2:
                            found = True
                            break
                    if found: continue

                    self.edges.append(Edge(p1,p2))

    def constrain(self, n):
        for constraint_pass in range(n):
            for edge in self.edges:
                edge.constrain()

    def update(self,dt):
        #Gravity
        for row in self.particles:
            for particle in row:
                particle.accel = [0.0, gravity, 0.0]

        #Move everything
        for row in self.particles:
            for particle in row:
                particle.move(dt)
                
    def update_mouse_xy(self,dt,mouse_rel):
        #Gravity
        for row in self.particles:
            for particle in row:
                particle.accel = [0.0, gravity, 0.0]
                

        # We will move some of the constrained particles with the mouse movements
        # Hence, we set their acc. and pos. updates manually here since 
        # the constrained particles are not moved normally in the particle class.
        self.particles[         0][         0].accel = [mouse_rel[0]/4.0, -mouse_rel[1]/4.0, 0.0]
        self.particles[ point_n-1][         0].accel = [mouse_rel[0]/4.0, -mouse_rel[1]/4.0, 0.0]  

        for i in [0,1,2]:
            self.particles[         0][         0].last_pos[i] = self.particles[         0][         0].pos[i]
            self.particles[ point_n-1][         0].last_pos[i] = self.particles[ point_n-1][         0].pos[i]

            self.particles[         0][         0].pos[i]      = self.particles[         0][         0].pos[i] + self.particles[         0][         0].accel[i]*dt
            self.particles[ point_n-1][         0].pos[i]      = self.particles[ point_n-1][         0].pos[i] + self.particles[ point_n-1][         0].accel[i]*dt                   
        
        #Move everything else (i.e the non-constrained )
        for row in self.particles:
            for particle in row:
                particle.move(dt)


    def draw(self):
        glColor3f(0.8,0.8,0) # select yellow color
        glPointSize(4) # each particle size

        glBegin(GL_POINTS)
        for row in self.particles:
            for particle in row:
                glVertex3fv(particle.pos)
        glEnd()

    def draw_wireframe(self):
        glColor3f(0,0.2,0) # select dark green color

        glBegin(GL_LINES)
        for edge in self.edges:
            glVertex3fv(edge.p1.pos)
            glVertex3fv(edge.p2.pos)
        glEnd()

    def draw_mesh(self):
        glColor3f(0.5,0.5,1.0) # select white color
        for z in range(self.res-1):
            glBegin(GL_QUAD_STRIP)
            for x in range(self.res):
                glVertex3fv(self.particles[z  ][x].pos)
                glVertex3fv(self.particles[z+1][x].pos)
            glEnd()

    def draw_rviz(self):
        clothRVIZPoints = []

        for row in self.particles:
            for particle in row:
                p = geometry_msgs.msg.Point()
                p.x = particle.pos[2]
                p.y = particle.pos[0]
                p.z = particle.pos[1]

                clothRVIZPoints.append(p)
        
        # Publish
        publish_rviz_points(clothRVIZPoints)

    def draw_rviz_wireframe(self):
        clothRVIZEdges = []

        for edge in self.edges:
            p1 = geometry_msgs.msg.Point()
            p1.x = edge.p1.pos[2]
            p1.y = edge.p1.pos[0]
            p1.z = edge.p1.pos[1]

            clothRVIZEdges.append(p1)

            p2 = geometry_msgs.msg.Point()
            p2.x = edge.p2.pos[2]
            p2.y = edge.p2.pos[0]
            p2.z = edge.p2.pos[1]

            clothRVIZEdges.append(p2)
        
        # Publish
        publish_rviz_lines(clothRVIZEdges)
    

point_n = 12
fabric_density = 200 # kg/m^2 (denim 0.3kg/m2)
fabric_area = 1.0 # m^2
total_num_particles = point_n*point_n
particle_mass = (fabric_density * fabric_area) / total_num_particles # kg
gravity = -9.80665*particle_mass # as force on each particle

cloth = ClothCPU(point_n)
target_fps = 90
dt = 1.0/float(target_fps)

camera_rot = [70,23]
camera_radius = 2.5
camera_center = [0.5,0.5,0.5]

def get_input():
    global camera_rot, camera_radius
    keys_pressed = pygame.key.get_pressed()
    mouse_buttons = pygame.mouse.get_pressed()
    mouse_rel = pygame.mouse.get_rel()

    for event in pygame.event.get():
        if   event.type == QUIT: return False
    
        elif event.type == KEYDOWN:
            if   event.key == K_ESCAPE: return False # to quit, press esc
            elif event.key == K_r: cloth.reset() # to reset, press r
    
        elif event.type == MOUSEBUTTONDOWN: # mouse scroll zoom in/out
            if   event.button == 4: camera_radius -= 0.5
            elif event.button == 5: camera_radius += 0.5

    if mouse_buttons[1]: # Middle click for view orientation change
        camera_rot[0] += mouse_rel[0]
        camera_rot[1] += mouse_rel[1]
    
    if mouse_buttons[0]: # Left click for object manipulation in xz
        cloth.constrain(3)
        cloth.update_mouse_xy(dt,mouse_rel)
    else:
        cloth.constrain(3)
        cloth.update(dt)

    return True

        
def draw():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    glViewport(0,0,screen_size[0],screen_size[1])
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45,float(screen_size[0])/float(screen_size[1]), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    camera_pos = [
        camera_center[0] + camera_radius*cos(radians(camera_rot[0]))*cos(radians(camera_rot[1])),
        camera_center[1] + camera_radius                            *sin(radians(camera_rot[1])),
        camera_center[2] + camera_radius*sin(radians(camera_rot[0]))*cos(radians(camera_rot[1]))
    ]
    gluLookAt(
        camera_pos[0],camera_pos[1],camera_pos[2],
        camera_center[0],camera_center[1],camera_center[2],
        0,1,0
    )

    # Draw the guiding cube
    glColor3f(1,0,0) # select red color
    glBegin(GL_LINES)
    points = []
    for x in [0,1]:
        for y in [0,1]:
            for z in [0,1]:
                points.append([x,y,z])
    for p1 in points:
        for p2 in points:
            unequal = sum([int(p1[i]!=p2[i]) for i in [0,1,2]])
            if unequal == 1:
                if p1 == [0,0,0]:
                    vect = subtract(p2,p1)
                    if vect == [1,0,0]: # x-axis
                        glColor3f(1,0,0) # select red color
                    elif vect == [0,1,0]: # y-axis
                        glColor3f(0,1,0) # select green color
                    elif vect == [0,0,1]: # z-axis
                        glColor3f(0,0,1) # select blue color
                else:
                    glColor3f(0.9,0.9,0.9) 
                glVertex3fv(p1)
                glVertex3fv(p2)
    glEnd()
        
    cloth.draw() # Draw mesh vertices 
    cloth.draw_rviz() # Publishes visualization points to ROS

    cloth.draw_wireframe() 
    cloth.draw_rviz_wireframe() # Publishes visualization edges to ROS

    cloth.draw_mesh() # draw fabric mesh
    
    pygame.display.flip()
    
    


def main():
    
    t_all=[]
    clock = pygame.time.Clock()
    
    while not rospy.is_shutdown():
    #for i in range(1000):
        tic = timeit.default_timer()
        if not get_input(): rospy.signal_shutdown("reason: User wanted to exit.")

        pos = []
        for x in range(point_n):
            for y in range(point_n):
                    pos.append(np.array(cloth.particles[x][y].pos))     
                    
        pos = np.array(pos)          
        # print(str(pos.flatten('C')))
        # return list(pos.flatten('C'))
        
        draw()
        clock.tick(target_fps)
        t_all.append(timeit.default_timer()-tic)
        print(timeit.default_timer()-tic)
        # Note: No need to add rospy.rate sleep here, let the loop run as fast as it can
        
    #print (np.mean(t_all),np.std(t_all))    
    pygame.quit()
    
if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
        pygame.quit()
        input()