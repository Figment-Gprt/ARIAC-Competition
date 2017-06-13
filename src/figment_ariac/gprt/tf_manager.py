#!/usr/bin/env python


import rospy, sys
from tf2_msgs.msg import TFMessage
from transform import Transform



class TfManager:
    """
        This class manages all transform objects published on the /tf topic
    """

    def __init__(self, timeBuffer=5):
        self.transforms_dynamic = {}
        self.transforms_static = {}
        self.timeBuffer = timeBuffer
        self.graph = Graph()
        self.part_id_black_list = []


    def __create_transform(self, translation, rotation):
        pos = [translation.x, translation.y, translation.z]
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        return Transform(pos, quat)

    def add_part_id_to_bl(self, part_id):
        self.part_id_black_list.append(part_id)

    def get_piece_tf_time(self, father, child):
        if len(father) > 0:
            return self.transforms_dynamic[father][child]['secs']

    def find_part_name(self, part_name, dad=None, sub_dad=None):
        # 
        father = ""
        child = ""
        if dad is not None and dad in self.transforms_dynamic:
            for k_child in self.transforms_dynamic[dad].keys():
                if part_name in k_child and k_child not in self.part_id_black_list:
                    child = k_child
                    father = dad
                    break
        elif sub_dad is not None:            
            fdad = ''
            # rospy.loginfo("[find_part_name] transforms_dynamic: " + str(self.transforms_dynamic))
            debug = True
            # rospy.loginfo("[find_part_name] DEBUG: ")
            # if(debug):
            #     for k in self.transforms_dynamic.keys():   
            #         rospy.loginfo("\n\n\n[find_part_name] k: " + str(k))
            #         values = self.transforms_dynamic[k]
            #         rospy.loginfo("[find_part_name] values: " + str(values) + "\n\n") 
            found = False 
            for k in self.transforms_dynamic.keys():
                rospy.loginfo("[find_part_name] test fdad/k: " + str(sub_dad) + "/" + str(k))  
                if sub_dad in k:
                    rospy.loginfo("[find_part_name] found sub_dad/k: " + str(sub_dad) + "/" + str(k))
                    fdad = k
                    for k_child in self.transforms_dynamic[fdad].keys():
                        if part_name in k_child and k_child not in self.part_id_black_list:
                            child = k_child
                            father = fdad
                            found = True
                            rospy.loginfo("[find_part_name] found child/father: " + str(child) + "/" + str(father))
                            break
                    if(found):
                        break                     

        if len(child) == 0 and sub_dad is None:
            for k in self.transforms_dynamic.keys():
                for k_child in self.transforms_dynamic[k].keys():
                    if part_name in k_child and k_child not in self.part_id_black_list:
                        child = k_child
                        break

                if len(child) > 0:
                    father = k
                    break

        if len(child) == 0 and sub_dad is None: #TODO check id we should avoid in case of sub_dad
            for k in self.transforms_static.keys():
                for k_child in self.transforms_static[k].keys():
                    if part_name in k_child and k_child not in self.part_id_black_list:
                        child = k_child
                        break

                if len(child) > 0:
                    father = k
                    break            


        return father, child

    def get_transform(self, father, child):
        t = None
        if father in self.transforms_dynamic:
            # rospy.loginfo(father + " is in transforms_dynamic")
            if child in self.transforms_dynamic[father]:
                # rospy.loginfo(child + " is in transforms_dynamic[" + father + "]")
                t = self.transforms_dynamic[father][child]['transform']

        if t is None:
            if father in self.transforms_static:
                # rospy.loginfo(father + " is in transforms_static")
                if child in self.transforms_static[father]:
                    # rospy.loginfo(child + " is in transforms_static[" + father + "]")
                    t = self.transforms_static[father][child]['transform']                

        return t


    def get_tf_object(self, father, child):
        t = None
        if father in self.transforms_dynamic:
            # rospy.loginfo(father + " is in transforms_dynamic")
            if child in self.transforms_dynamic[father]:
                # rospy.loginfo(child + " is in transforms_dynamic[" + father + "]")
                t = self.transforms_dynamic[father][child]

        if t is None:
            if father in self.transforms_static:
                # rospy.loginfo(father + " is in transforms_static")
                if child in self.transforms_static[father]:
                    # rospy.loginfo(child + " is in transforms_static[" + father + "]")
                    t = self.transforms_static[father][child]

        return t



    def get_transform_list(self, target, base, time):
        _list = self.graph.find_path(target, base)
        transform_list = []
        father = ""
        child = _list.pop(0)
        while len(_list) > 0:
            father = _list.pop(0)
            tf_object = self.get_tf_object(father, child)
            if 'secs' in tf_object:
                # rospy.loginfo("[tf_manager] tf[" + father + "][" + child + "][secs] = " + str(tf_object['secs']) + " expected time = " + str(time) )
                if tf_object['secs'] >= time:
                    transform_list.append(tf_object['transform'])
                    child = father
                else:
                    _list.insert(0, father)
            else:
                transform_list.append(tf_object['transform'])
                child = father  

        #rospy.loginfo("[TfManager]: Transforms list: " + str(transform_list))
        return transform_list



    def remove_transform(self, father, child):
        if father in self.transforms_dynamic:
            if child in self.transforms_dynamic[father]:
                del self.transforms_dynamic[father]

        if father in self.transforms_static:
            if child in self.transforms_static[father]:
                del self.transforms_static[father][child]


    def tf_callback(self, msg):
        frames_ids = []
        newestSec = -sys.maxint - 1
        for frame in msg.transforms:
            transform = self.__create_transform(
                frame.transform.translation, frame.transform.rotation)
            frames_ids.append(frame.header.frame_id)
            if frame.header.frame_id not in self.transforms_dynamic:
                self.transforms_dynamic[frame.header.frame_id] = {}
                self.graph.addNode(frame.header.frame_id)

            if frame.child_frame_id not in self.transforms_dynamic[frame.header.frame_id]:
                self.transforms_dynamic[frame.header.frame_id][frame.child_frame_id] = {
                    'hash': transform.hash_value(),
                    'transform': transform,
                    'secs' : frame.header.stamp.secs

                }
                self.graph.addNode(frame.child_frame_id)
                self.graph.addEdge(frame.child_frame_id, frame.header.frame_id)


            elif self.transforms_dynamic[frame.header.frame_id][frame.child_frame_id]['hash'] != transform.hash_value():
                self.transforms_dynamic[frame.header.frame_id][frame.child_frame_id] = {
                    'hash': transform.hash_value(),
                    'transform': transform,
                    'secs' : frame.header.stamp.secs

                }

            elif self.transforms_dynamic[frame.header.frame_id][frame.child_frame_id]['hash'] == transform.hash_value():
                self.transforms_dynamic[frame.header.frame_id][
                    frame.child_frame_id]['secs'] = frame.header.stamp.secs
            
            newestSec = max(newestSec, frame.header.stamp.secs)
            

        for k in self.transforms_dynamic.keys():
            for j in self.transforms_dynamic[k].keys():
                if newestSec - self.transforms_dynamic[k][j]['secs'] > self.timeBuffer:
                    del self.transforms_dynamic[k][j]
                    self.graph.delNode(j)
                    
                
                


    def tf_static_callback(self, msg):
        for frame in msg.transforms:
            transform = self.__create_transform(
                frame.transform.translation, frame.transform.rotation)

            if frame.header.frame_id not in self.transforms_static:
                self.transforms_static[frame.header.frame_id] = {}
                self.graph.addNode(frame.header.frame_id)

            if frame.child_frame_id not in self.transforms_static[frame.header.frame_id]:
                self.transforms_static[frame.header.frame_id][frame.child_frame_id] = {
                    'hash': transform.hash_value(),
                    'transform': transform
                }
                self.graph.addNode(frame.child_frame_id)
                self.graph.addEdge(frame.child_frame_id, frame.header.frame_id)


class Graph:
    """
    Graph class used as path finder between transforms
    """

    def __init__(self):
        self._nodes = []
        self._graph = {}


    def addNode(self, node):
        if node not in self._nodes:
            self._nodes.append(node)
            self._graph[node] = []


    def delNode(self, node):
        self._nodes.remove(node)
        del self._graph[node]

    def addEdge(self, nodeA, nodeB, directed=True):
        """
            Add an edge between Node A and Node B. 
            Default behavior is the edge to be directed.

            nodeA - node A name
            nodeB - node B name

            return true if added
        """
        if nodeA in self._nodes and nodeB in self._nodes:
            if nodeB not in self._graph[nodeA]:
                self._graph[nodeA].append(nodeB)
                return True

        return False


    def find_path(self, nodeA, nodeB):
        """
            Find path from node A to node B
        """
        #rospy.loginfo("[TfManager]: Finding Path from " + str(nodeA) + " to " + str(nodeB))
        finalNode = ""
        path = [nodeA]
        stack = list(self._graph[nodeA])
        while len(stack) > 0:
            finalNode = stack.pop()
            #rospy.loginfo("[TfManager]: Node " + str(finalNode) + " in path")
            path.append(finalNode)
            if finalNode != nodeB:
                stack.extend(self._graph[finalNode])

        #rospy.loginfo("[TfManager]: Final List: " + str(path))
        return path if finalNode == nodeB else []
            





