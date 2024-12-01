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
        self.node.create_subscription(String, '/signal_states', self.update_flags, 10)
    
    def update_flags(self,msg):
        self.flags = json.loads(msg.data)

    def check_flags(self, wf):
        for f in wf:
            if f not in self.flags:
                return False
        return True

    def exec(self, params):
        # It needs params["waiting_string"]
        wait_str = params["waiting_string"]
        wait_flags = wait_str.split('&')

        while not self.check_flags(wait_flags):
            rclpy.spin(self.node)
            time.sleep(1)


class SendSkill():
    def __init__(self,node) -> None:
        self.node = node
        self.publisher = self.node.create_publisher(String, "/mission_signals",10)
    
    def exec(self, params):
        # It needs params["task_id"]
        task_id = params["task_id"]
        d = {"task_id": task_id}
        self.publisher.publish(String(json.dumps(d)))
        



class VMopSkill():
    def __init__(self,node) -> None:
        # Starting skill : vmop
        self.node.get_logger().info("Starting up skill: VMop")
    
    def exec(self, params):
        # It needs: params["room"]["size"]
        rsize = params["room"]["size"]
        time.sleep(rsize*1.5)
        return True


# Skill Manager


class SkillManager():
    def __init__(self) -> None:
        self.sk_map = {
            "wait": WaitSkill,
            "send": SendSkill,
            "mop": VMopSkill
        }

    def skill_map(self):
        return self.sk_map
