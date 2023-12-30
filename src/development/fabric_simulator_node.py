#!/usr/bin/env python
"""
Python implementation of XpBD cloth simulation integrated with ROS.
Author: Burak Aksoy
Note: "solve" functions during the simulations require iterative for loops for each edge/particle.
This creates a bottle neck for the performance of the simulation. Insipired Java Script implementation: https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/14-cloth.html does not suffer from this problem. 
I will imlement a c++ version of this code to improve. 
"""
from __future__ import absolute_import

import rospy

import visualization_msgs.msg # for Markers
import geometry_msgs.msg
from std_msgs.msg import Float32 # For reporting simulation timings
import nav_msgs.msg # To subscribe odom msgs

import sys, os, traceback

import math
import numpy as np
import time

import tabulate
from scipy import spatial # used to find nearest attachment particle pose

import threading

class Cloth():
    def __init__(self, mesh, bending_compliance=1.0, density=1.0):
        # particles
        self.num_particles = len(mesh["vertices"])
        self.pos = mesh["vertices"]
        self.prev_pos =self.pos
        self.rest_pos = self.pos
        self.vel = np.zeros(self.pos.shape, dtype=np.float32)
        self.inv_mass = np.zeros(self.num_particles, dtype=np.float32)

        self.density = density # fabric mass per meter square (kg/m^2)
        self.attached_ids = [] # ids of robot attached particles

        # stretching and bending constraints
        neighbors = self.find_tri_neighbors(mesh["face_tri_ids"])
        num_tris = len(mesh["face_tri_ids"])
        edge_ids = []
        tri_pair_ids = []
        for i in range(num_tris):
            for j in range(3):
                global_edge_nr = 3 * i + j
                # ids of particle creating that global_edge:
                id0 = mesh["face_tri_ids"][i][j]
                id1 = mesh["face_tri_ids"][i][(j + 1) % 3]

                # Each edge only once
                n = neighbors[global_edge_nr] # returns -1 if open edge, or positive global edge number of its pair
                if n < 0 or id0 < id1:
                    edge_ids.append([id0, id1])

                # Tri pair
                if n >= 0 and id0<id1:
                    ni = n // 3
                    nj = n % 3
                    id2 = mesh["face_tri_ids"][i][(j + 2) % 3]
                    id3 = mesh["face_tri_ids"][ni][(nj + 2) % 3]
                    tri_pair_ids.append([id0, id1, id2, id3]) # simple bending constraint needs only id2 and id3, but id0 and id1 also added for a "future" implementation

        self.stretching_ids = np.array(edge_ids, dtype=np.int32) # all id pairs of particles that create unique edges. NOTE: Use this to draw edges on RVIZ.
        # print("self.stretching_ids ",self.stretching_ids)
        self.bending_ids = np.array(tri_pair_ids, dtype=np.int32) # all ids of particle that will be used in bending constraints
        # print("self.bending_ids ", self.bending_ids)

        self.stretching_lengths = np.zeros(len(self.stretching_ids), dtype=np.float32) # we pushed 2 ids, hence divide by 2 for size
        self.bending_lengths = np.zeros(len(self.bending_ids), dtype=np.float32) # we pushed 4 ids, hence divide by 4 for size

        self.stretching_compliance = 0.0
        self.bending_compliance = bending_compliance

        self.grads = np.zeros(3, dtype=np.float32)

        self.grab_id = -1
        self.grab_inv_mass = 0.0

        self.init_physics(mesh["face_tri_ids"])

        self.only_once = 0

    def find_tri_neighbors(self,triIds):
        """
        This explanation is created by chatGPT:
        This is a JavaScript function that finds the neighboring triangles of a set of 
        triangles in a 3D space. Given an input array "triIds" of length N, where N is
        a multiple of 3, the function splits it into an array of "num_tris" triangles,
        where each triangle is represented by 3 vertex indices.

        The function then creates a set of edges, which connect the vertices of each 
        triangle, and stores these edges in an array called "edges". The edges are
        sorted so that common edges between triangles are next to each other in the 
        "edges" array.

        Next, the function finds matching edges by checking if the vertices of one
        edge match those of the next edge in the sorted "edges" array. If a matching
        edge is found, the triangle index of the two edges are stored in a "neighbors"
        array, indicating that they are neighbors. If no matching edge is found, the
        value in the "neighbors" array is set to -1, indicating an open edge.

        Finally, the function returns the "neighbors" array.
        """
        
        num_tris = len(triIds)

        # create common edges
        edges = []
        for i in range(num_tris):
            for j in range(3):
                id0 = triIds[i][j]
                id1 = triIds[i][(j + 1) % 3]
                edges.append({
                    "id0": min(id0, id1),
                    "id1": max(id0, id1),
                    "edgeNr": 3 * i + j
                })

        # print ("edges")
        # header = edges[0].keys()
        # rows =  [x.values() for x in edges]
        # print(tabulate.tabulate(rows, header))

        # sort so common edges are next to each other
        edges = sorted(edges, key=lambda x: (x["id0"], x["id1"]))

        # print ("sorted edges")
        # header = edges[0].keys()
        # rows =  [x.values() for x in edges]
        # print(tabulate.tabulate(rows, header))

        # find matching edges
        neighbors = [-1] * (3 * num_tris)

        nr = 0
        while nr < len(edges):
            e0 = edges[nr]
            nr += 1
            if nr < len(edges):
                e1 = edges[nr]
                if e0["id0"] == e1["id0"] and e0["id1"] == e1["id1"]:
                    neighbors[e0["edgeNr"]] = e1["edgeNr"]
                    neighbors[e1["edgeNr"]] = e0["edgeNr"]

        # print (neighbors)
        return neighbors


    def init_physics(self,tri_ids):
        num_tris = len(tri_ids)

        for i in range(num_tris):
            id0 = tri_ids[i][0]
            id1 = tri_ids[i][1]
            id2 = tri_ids[i][2]
            
            e0 = self.pos[id1] - self.pos[id0]
            e1 = self.pos[id2] - self.pos[id0]
            c = np.cross(e0,e1)
        
            A = 0.5 * np.linalg.norm(c) # area
            mass = A * self.density 
            # p_mass = (mass / 3.0) if mass > 0.0 else 0.0 # divide by 3 because we have 3 vertices per triangle

            if mass > 0.0:
                p_mass = (mass / 3.0)  # divide by 3 because we have 3 vertices per triangle

                p_0_mass = 1.0/self.inv_mass[id0] if self.inv_mass[id0] > 0.0 else 0.0
                p_1_mass = 1.0/self.inv_mass[id1] if self.inv_mass[id1] > 0.0 else 0.0
                p_2_mass = 1.0/self.inv_mass[id2] if self.inv_mass[id2] > 0.0 else 0.0

                p_0_mass += p_mass
                p_1_mass += p_mass
                p_2_mass += p_mass

                self.inv_mass[id0] = 1.0/p_0_mass
                self.inv_mass[id1] = 1.0/p_1_mass
                self.inv_mass[id2] = 1.0/p_2_mass

        print ("Total Fabric mass: " + str(np.sum(1./self.inv_mass)) + " kg")
        # print (1./self.inv_mass)

        for i in range(len(self.stretching_lengths)):
            id0 = self.stretching_ids[i][0]
            id1 = self.stretching_ids[i][1]
            self.stretching_lengths[i] = np.linalg.norm(self.pos[id1] - self.pos[id0])

        for i in range(len(self.bending_lengths)):
            id0 = self.bending_ids[i][2]
            id1 = self.bending_ids[i][3]
            self.bending_lengths[i] = np.linalg.norm(self.pos[id1] - self.pos[id0])

        self.hang_from_corners()
        
    def pre_solve(self, dt, gravity):        
        ids = np.nonzero(self.inv_mass)
        self.vel[ids] += gravity*dt
        self.prev_pos[ids] = self.pos[ids]
        self.pos[ids] += self.vel[ids]*dt

        # Prevent going below ground
        ids = np.flatnonzero(self.pos[:,2] < 0)
        self.pos[ids] = self.prev_pos[ids]
        self.pos[ids,2] = 0.0
        

    def solve(self, dt):
        self.solve_stretching(self.stretching_compliance, dt)
        self.solve_bending(self.bending_compliance, dt)

    def post_solve(self, dt):
        ids = np.nonzero(self.inv_mass)
        self.vel[ids] = (self.pos[ids] - self.prev_pos[ids])/dt


    def solve_stretching(self, compliance, dt):
        alpha = compliance / (dt**2)

        id0s = self.stretching_ids[:,0]
        id1s = self.stretching_ids[:,1]
        w0s = self.inv_mass[id0s]
        w1s = self.inv_mass[id1s]

        ws = w0s + w1s
        rows = np.nonzero(ws)

        # update useful ids, inverse masses, rest lengths
        id0s = id0s[rows]
        id1s = id1s[rows]
        w0s = w0s[rows]
        w1s = w1s[rows]
        ws = ws[rows]
        rest_lens = self.stretching_lengths[rows]

        for i in range(len(rest_lens)):
            grad = self.pos[id0s[i]] - self.pos[id1s[i]] 

            len_ = np.linalg.norm(grad)
            if len_ == 0.0:
                continue

            grad = (grad/len_)

            C = len_ - rest_lens[i]
            s = -C / (ws[i] + alpha)
            
            self.pos[id0s[i]] +=  s*w0s[i]*grad
            self.pos[id1s[i]] += -s*w1s[i]*grad


    def solve_bending(self, compliance, dt):
        alpha = compliance / (dt**2)

        for i in range(len(self.bending_lengths)):
            id0 = self.bending_ids[i][2]
            id1 = self.bending_ids[i][3]
            w0 = self.inv_mass[id0]
            w1 = self.inv_mass[id1]
            w = w0 + w1
            if w == 0.0:
                continue

            self.grads = self.pos[id0] - self.pos[id1] 

            len_ = np.linalg.norm(self.grads)
            if len_ == 0.0:
                continue

            self.grads = (self.grads/len_)  

            restLen = self.bending_lengths[i]
            C = len_ - restLen
            s = -C / (w + alpha)

            self.pos[id0] +=  s*w0*self.grads
            self.pos[id1] += -s*w1*self.grads

    def hang_from_corners(self):
        min_x = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for i in range(self.num_particles):
            min_x = min(min_x, self.pos[i][0])
            max_x = max(max_x, self.pos[i][0])
            max_y = max(max_y, self.pos[i][1])

        eps = 0.0001

        for i in range(self.num_particles):
            x = self.pos[i][0]
            y = self.pos[i][1]
            if y > max_y - eps and (x < min_x + eps or x > max_x - eps):
                self.inv_mass[i] = 0.0

    def attach_nearest(self,pos):
        """Attaches a given pos to nearest particle pose"""

        pos_tree = spatial.KDTree(self.pos)
        id = pos_tree.query(pos)[1] # nearest particle id

        # Make that particle stationary
        self.inv_mass[id] = 0.0

        self.attached_ids.append(id)
        return id

    def update_attached_pose(self, attached_id, pos):
        self.pos[attached_id] = np.array(pos)
        




class FabricSimulator():
    def __init__(self):
        rospy.init_node('fabric_sim_node', anonymous=True)
        self.sim_lock = threading.Lock()

        self.fabric_points_topic_name = rospy.get_param('~cloth_points_topic_name', "cloth_points")

        self.pub_fabric_points = rospy.Publisher(self.fabric_points_topic_name, visualization_msgs.msg.Marker, queue_size=1)


        # Physics scene parameters
        self.gravity = np.array([0.0, 0.0, -9.81]) # TODO: get this with param
        self.dt = 1.0 / 30.0 # TODO: get this with param
        self.num_substeps = rospy.get_param('~num_substeps', 3)
        self.num_steps = rospy.get_param('~num_steps', 1)
        self.sim_objects = []
        

        # Fabric mesh properties: (Assuming fabric is rectangular)
        self.fabric_x = rospy.get_param('~fabric_x', 2) # expansion in x dimension (m)
        self.fabric_y = rospy.get_param('~fabric_y', 2) # expansion in y dimension (m)
        # Average fabric density in factory noted as: 20 ounce per square foot --> ~ 6 kg/m^2
        self.fabric_density = rospy.get_param('~fabric_density', 1)  # fabric mass per meter square (kg/m^2)
        self.fabric_resolution = rospy.get_param('~fabric_resolution', 5) # particle resolution per meter 
        self.fabric_bending_compliance = rospy.get_param('~fabric_bending_compliance', 0.0) 
        
        self.initial_height = rospy.get_param('~initial_height', 3.0) # initial fabric height from ground (m)
        
        # Init Physics
        mesh = self.create_mesh_rectangular()
        # print(mesh)

        fabric = Cloth(mesh, bending_compliance=self.fabric_bending_compliance, density=self.fabric_density)
        self.sim_objects.append(fabric)

        self.time_frames = 0
        self.time_sum = 0.

        self.simulation_rate = rospy.get_param('~simulation_rate', 90.0)
        rospy.Timer(rospy.Duration(1.0/(self.simulation_rate)), self.simulate)

        self.rending_rate = rospy.get_param('~rending_rate', 30.0)
        rospy.Timer(rospy.Duration(1.0/self.rending_rate), self.render)

        # Odometry subscribers from robots
        # TODO: later make the nummber of subscribers more dynamic based on yaml file
        # TODO: Maybe add a service to spawn and delete these subsribers and accordingly attach/detach the cloth
        self.odom_01_topic_name = rospy.get_param('~odom_01_topic_name', "d1/ground_truth/odom")
        self.odom_02_topic_name = rospy.get_param('~odom_02_topic_name', "d2/ground_truth/odom")
        self.odom_03_topic_name = rospy.get_param('~odom_03_topic_name', "d3/ground_truth/odom")
        self.odom_04_topic_name = rospy.get_param('~odom_04_topic_name', "d4/ground_truth/odom")

        self.is_rob_01_attached = False
        self.is_rob_02_attached = False
        self.is_rob_03_attached = False
        self.is_rob_04_attached = False

        self.rob_01_attached_id = None
        self.rob_02_attached_id = None
        self.rob_03_attached_id = None
        self.rob_04_attached_id = None

        rospy.Subscriber(self.odom_01_topic_name,nav_msgs.msg.Odometry, self.odometry_cb_01, queue_size=1)
        # rospy.Subscriber(self.odom_02_topic_name,nav_msgs.msg.Odometry, self.odometry_cb_02, queue_size=1)
        # rospy.Subscriber(self.odom_03_topic_name,nav_msgs.msg.Odometry, self.odometry_cb_03, queue_size=1)
        # rospy.Subscriber(self.odom_04_topic_name,nav_msgs.msg.Odometry, self.odometry_cb_04, queue_size=1)

        self.fabric_rob_z_offset = rospy.get_param('~fabric_rob_z_offset', 1.0)

        
    def create_mesh_rectangular(self):
        fabric_x = self.fabric_x
        fabric_y = self.fabric_y
        fabric_z = self.initial_height
        fabric_res = self.fabric_resolution

        vertices = []
        face_tri_ids = []

        num_particle_x = fabric_x * fabric_res
        num_particle_y = fabric_y * fabric_res

        # assume a fabric centered at the origin
        x_coords = np.linspace(fabric_x/2.0,-fabric_x/2.0,num=num_particle_x+1)
        y_coords = np.linspace(fabric_y/2.0,-fabric_y/2.0,num=num_particle_y+1)

        print("x_coords ", x_coords)
        print("y_coords ", y_coords)
        
        # create vertices with x,y,z coordinates
        for x in x_coords:
            for y in y_coords:
                vertices.append([x,y,fabric_z])

        num_particles = len(vertices)/3

        # create face triangle ids
        id = 0
        for i in range(len(x_coords)-1): 
            for j in range(len(y_coords)-1): # creates one row opeations
                face_tri_ids.append([id,id+1,id + len(y_coords)])
                
                if j > 0:
                    face_tri_ids.append([id,id + len(y_coords),id + len(y_coords) - 1])

                if (j + 1) == (len(y_coords)-1):
                    face_tri_ids.append([id+1,id+1 + len(y_coords),id+1 + len(y_coords) - 1])
                    id = id + 1    
                id = id + 1

        mesh = { "name": "cloth",
                 "vertices": np.array(vertices,dtype=np.float32),
                 "face_tri_ids": np.array(face_tri_ids,dtype=np.int32)}

        return mesh

    def publish_rviz_points(self, points):
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

        self.pub_fabric_points.publish(m)

    def publish_rviz_lines(self, points):
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

        self.pub_fabric_points.publish(m)

    def draw_rviz(self,poses):
        clothRVIZPoints = []

        for pose in poses:
            p = geometry_msgs.msg.Point()
            p.x = pose[0]
            p.y = pose[1]
            p.z = pose[2]

            clothRVIZPoints.append(p)
        
        # Publish
        self.publish_rviz_points(clothRVIZPoints)

    def draw_rviz_wireframe(self,poses,ids):
        clothRVIZEdges = []

        for pair in ids:
            id0 = pair[0]
            id1 = pair[1]

            p1 = geometry_msgs.msg.Point()
            p1.x = poses[id0][0]
            p1.y = poses[id0][1]
            p1.z = poses[id0][2]

            clothRVIZEdges.append(p1)

            p2 = geometry_msgs.msg.Point()
            p2.x = poses[id1][0]
            p2.y = poses[id1][1]
            p2.z = poses[id1][2]

            clothRVIZEdges.append(p2)
        
        # Publish
        self.publish_rviz_lines(clothRVIZEdges)

    def simulate(self,event):
        with self.sim_lock:
            sdt = self.dt / self.num_substeps

            start_time = time.time()
            
            # Small steps implementation 
            # ------------------------------------------------
            for i in range(self.num_steps):
                for sub_step in range(self.num_substeps):
                    for object in self.sim_objects:
                        object.pre_solve(sdt, self.gravity)

                    for object in self.sim_objects:
                        object.solve(sdt)

                    for object in self.sim_objects:
                        object.post_solve(sdt)
            # ------------------------------------------------
            
            # # # ------------------------------------------------
            # for object in self.sim_objects:
            #     object.pre_solve(sdt, self.gravity)

            # for i in range(self.num_steps):
            #     for object in self.sim_objects:
            #         object.solve(sdt)

            # for object in self.sim_objects:
            #     object.post_solve(sdt)
            # # # ------------------------------------------------

            finish_time = time.time()

            self.time_sum += finish_time - start_time
            self.time_frames += 1

            if self.time_frames > 1:
                self.time_sum /= self.time_frames
                print("{:.3f}".format(round(self.time_sum, 3))  + " s per frame") 
                # TODO: publish above line as a debug msg later.
                self.time_frames = 0
                self.time_sum = 0.

    def render(self,event):
        with self.sim_lock:
            # Publish RVIZ visualization markers here
            for object in self.sim_objects:
                self.draw_rviz(object.pos)
                self.draw_rviz_wireframe(object.pos,object.stretching_ids)


    def odometry_cb_01(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z + self.fabric_rob_z_offset
        pos = [x,y,z]
        # print(pos)

        if not self.is_rob_01_attached:
            # tell sim objects (fabric) to attach robot to the nearest particles
            for object in self.sim_objects: # There is only one object now, and it is the fabric
                self.rob_01_attached_id = object.attach_nearest(pos)
                print("self.rob_01_attached_id, " + str(self.rob_01_attached_id))
            
            if self.rob_01_attached_id is not None:
                self.is_rob_01_attached = True
        else:
            # tell sim objects to update its position
            for object in self.sim_objects:
                object.update_attached_pose(self.rob_01_attached_id, pos)

    def odometry_cb_02(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z + self.fabric_rob_z_offset
        pos = [x,y,z]

        if not self.is_rob_02_attached:
            # tell sim objects (fabric) to attach robot to the nearest particles
            for object in self.sim_objects: # There is only one object now, and it is the fabric
                self.rob_02_attached_id = object.attach_nearest(pos)
                print("self.rob_02_attached_id, " + str(self.rob_02_attached_id))
            
            if self.rob_02_attached_id is not None:
                self.is_rob_02_attached = True
        else:
            # tell sim objects to update its position
            for object in self.sim_objects:
                object.update_attached_pose(self.rob_02_attached_id, pos)
    
    def odometry_cb_03(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z + self.fabric_rob_z_offset
        pos = [x,y,z]

        if not self.is_rob_03_attached:
            # tell sim objects (fabric) to attach robot to the nearest particles
            for object in self.sim_objects: # There is only one object now, and it is the fabric
                self.rob_03_attached_id = object.attach_nearest(pos)
                print("self.rob_03_attached_id, " + str(self.rob_03_attached_id))
            
            if self.rob_03_attached_id is not None:
                self.is_rob_03_attached = True
                
        else:
            # tell sim objects to update its position
            for object in self.sim_objects:
                object.update_attached_pose(self.rob_03_attached_id, pos)

    def odometry_cb_04(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z + self.fabric_rob_z_offset
        pos = [x,y,z]

        if not self.is_rob_04_attached:
            # tell sim objects (fabric) to attach robot to the nearest particles
            for object in self.sim_objects: # There is only one object now, and it is the fabric
                self.rob_04_attached_id = object.attach_nearest(pos)
                print("self.rob_04_attached_id, " + str(self.rob_04_attached_id))
            
            if self.rob_04_attached_id is not None:
                self.is_rob_04_attached = True
        else:
            # tell sim objects to update its position
            for object in self.sim_objects:
                object.update_attached_pose(self.rob_04_attached_id, pos)




if __name__ == '__main__':
    fabricSimulator = FabricSimulator()
    rospy.spin()