import rclpy
import time
from std_msgs.msg import String
import json
import numpy as np
# It includes always wait and send as base skills

# Skills
class WaitSkill():
    def __init__(self,node) -> None:
        self.node = node
        self.subs = self.node.create_subscription(String, '/signal_states', self.update_flags, 10)
    
    def update_flags(self,msg):
        self.flags = json.loads(msg.data)

    def check_flags(self, wf):
        for f in wf:
            if f not in self.flags:
                return False
        return True

    def exec(self, params):
        # It needs params["target"]
        wait_str = params["target"]
        wait_flags = wait_str.split('&')

        while not self.check_flags(wait_flags):
            rclpy.spin(self.node)
            time.sleep(1)
        return True
    
    def destroy(self):
        self.node.destroy_subscription(self.subs)


class SendSkill():
    def __init__(self,node) -> None:
        self.node = node
        self.publisher = self.node.create_publisher(String, "/mission_signals",10)
    
    def exec(self, params):
        # It needs params["target"]
        task_id = params["target"]
        msg = String()
        msg.data = task_id
        self.publisher.publish(msg)
        return True
        

class VMopSkill():
    def __init__(self,node) -> None:
        # Starting skill : vmop
        self.node = node
        self.node.get_logger().info("Starting up skill: VMop")
    
    def exec(self, params):
        # It needs: params["room"]["size"]
        print("Received the params: ",params)
        rsize = params["size"]
        time.sleep(rsize*1.5)
        self.node.get_logger().info("Finishing up running skill: VMop")
        return True

class VVacuumSkill():
    def __init__(self,node) -> None:
        # Starting skill : vmop
        self.node = node
        self.node.get_logger().info("Starting up skill: VVacuum")
    
    def exec(self, params):
        # It needs: params["room"]["size"]
        rsize = params["size"]
        time.sleep(rsize*2.5)
        self.node.get_logger().info("Finishing up running skill: VMop")
        return True


# Skill Manager


class SkillManager():
    def __init__(self, skill_mask) -> None:
        """
        skill_mask: if specified, then only those skills are created inside the SKmanager
        """
        self.sk_map = {
            "wait_until": WaitSkill,
            "send_signal": SendSkill,
            "mop": VMopSkill,
            "vacuum": VVacuumSkill
        }
        self.sk_map = self.filter_skills(self.sk_map,skill_mask)
    
    def filter_skills(self, sk_map, mask):
        if mask == "-": # If no skill mask provided, we left it intact
            return sk_map
        new_sk_map = {}
        sk_list = mask.split(",")
        for k,v in sk_map.items():
            if k in sk_list or k=="send_signal" or k=="wait_until":
                new_sk_map[k] = v
        return new_sk_map

    def skill_map(self):
        return self.sk_map
