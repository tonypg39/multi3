# Coordinator Tasks
# - Listen to /mission_comms and update the flag_table
# assign the available fragment to idle robots
import json
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from multi3_interfaces.srv import Fragment
from ament_index_python import get_package_prefix

class CoordinatorNode(Node):
    def __init__(self, fragments):
        super().__init__("multi3_coordinator")
        self.coord_settings = {
            "signal_states_period": 2.0,
            "assignment_period": 4.0
        }
        # FIXME: Move these dicts to a JSON, specially the second
        self.robot_inventory = {
            "robot1": ["mop","clean_mop"],
            "robot2": ["mop","vacuum"],
        }
        self.idle_robots = {}
        self.signal_states = ['SYSTEM_START']
        self.create_subscription(String, "/mission_signals",self.update_signal_state,10)
        self.create_subscription(String, "/hb_broadcast",self.update_hb,10)
        self.signal_publisher = self.create_publisher(String, '/signal_states', 10)
        self.signal_pub_timer = self.create_timer(self.coord_settings['signal_states_period'],self.broadcast_signal_states)
        self.assignment_timer = self.create_timer(self.coord_settings['assignment_period'],self.assign)
        self.fragments = self.load_fragments(fragments)
        # Dict to keep track of the states of executed fragments 
        # (For a frag to have a key here, the fragment must've been sent )
        self.fragments_futures = {} 
        # self.assign()
        # self.get_logger().info(len(self.fragments))
        

    
    def load_fragments(self, fragments):
        c = 0
        F = {}
        for fr in fragments:
            fr_obj = {
                "status": "waiting",
                "fragment_id": f"fr_{c}",
                "age": 0,
            }
            fr_obj.update(fr)
            F[fr_obj["fragment_id"]] = fr_obj
            c += 1
        return F

    def update_hb(self, msg):
        st = msg.data.split("=")
        if st[1] == "idle":
            self.idle_robots[st[0]] = True
        else:
            self.idle_robots[st[0]] = False
        


    def update_signal_state(self, msg):
        new_signal = msg.data
        if new_signal not in self.signal_states:
            self.signal_states.append(new_signal)
    
    
    # Publish the signal_states periodically using a Timer object
    def broadcast_signal_states(self):
        message = String()
        message.data = json.dumps(self.signal_states)
        # self.get_logger().info(f"Sending signals info: {message.data}")
        self.signal_publisher.publish(message)
    
    def check_active_fragments(self):
        active_frags = []
        for frag in self.fragments:
            if frag['status'] == "waiting":
                active_frags.append(frag)
            if frag['status'] == "blocked":
                w_flags = frag["initial_wait"].split('&')
                missing_flag = False
                for f in w_flags:
                    if f not in self.signal_states:
                        missing_flag = True
                if not missing_flag:
                    self.fragments[frag["fragment_id"]]["status"] = "waiting"
                    active_frags.append(frag)
        return active_frags
    
    def check_eligibility(self, robot,fragment):
        able = True        
        for t in fragment["tasks"]:
            if t["id"].find("|") > -1: # If it is either a signal or a wait 
                continue
            if t["id"] not in self.robot_inventory[robot]:
                able = False
        self.get_logger().info(f"Checking elegibility for {robot} and tasks {fragment['tasks']} = {able}")
        return able
    
    def generate_assigments(self, robots, fragments):
        assignment_dict = {}
        sorted_frags = sorted(fragments, key= lambda x: x["age"], reverse=True)
        for f in sorted_frags:
            for r in robots:
                if self.check_eligibility(r,f):
                    self.fragments[f["fragment_id"]]["status"] = "executed"
                    assignment_dict[r] = f.copy()
                    break
        return assignment_dict
            

        
    def get_idle_robots(self):
        # get available services 
        ir = []
        for k,v in self.idle_robots.items():
            if v:
                ir.append(k)
        return ir
    
    def get_active_fragments(self):
        active_frags = []
        for frag in self.fragments.values():
            if frag["status"] == "waiting":
                w_flags = frag['initial_wait'].split('&')
                missing_signal = any(fl not in self.signal_states for fl in w_flags)
                if not missing_signal:
                    active_frags.append(frag)
        return active_frags
    
    def send_assignment(self, robot, fragment):
        req = Fragment.Request()
        req.fragment = json.dumps(fragment)
        cli = self.create_client(Fragment, f'/{robot}/get_fragment')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.fragments_futures[fragment["fragment_id"]] = cli.call_async(req)
        


    def assign(self):
        robots = self.get_idle_robots()
        fragments = self.get_active_fragments()
        self.get_logger().info("\n\n------Assignment window------")
        self.get_logger().info(f"The active fragments are: {fragments}")
        self.get_logger().info(f"The active robots are: {robots}")

        assignments = self.generate_assigments(robots, fragments)
        # print(assignments)
        
        for k,v in assignments.items():
            print("Sending assignment: ", k,v)
            self.send_assignment(k,v)
        # Increase the age of the unpicked fragments
        for f in fragments:
            if self.fragments[f["fragment_id"]]["status"] == "waiting":
                self.fragments[f["fragment_id"]]["age"] += 1



def read_fragments():
    package_path = get_package_prefix("multi3_coordinator").replace("install","src")
    # print(package_path)
    with open(f"{package_path}/multi3_coordinator/tasks.json") as f:
        frags = json.load(f)
    return frags

def main(args=None):
    rclpy.init(args=args)
    fragments = read_fragments()
    coord = CoordinatorNode(fragments)
    rclpy.spin(coord)
    coord.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()