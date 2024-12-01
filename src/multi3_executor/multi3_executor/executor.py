from rclpy.node import Node
from .skills import SkillManager


class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        sk_mg = SkillManager()
        self.sk_map = sk_mg.skill_map()
        

    def exec(self, frag):
        #FIXME: Make a service unavailable
        #...
        failure = False
        for t in frag["tasks"]:
            sk = self.sk_map[t["id"]](self)
            failure |= sk.exec(t["vars"])
        
        if not failure:
            # If result of the Mission is successful make service avail again
            pass 
    
    


    
