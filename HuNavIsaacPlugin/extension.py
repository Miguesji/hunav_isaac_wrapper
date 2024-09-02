import asyncio
import gc
import omni.ext
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType
from pxr import UsdGeom, Gf, UsdSkel, UsdPhysics, PhysxSchema, Vt
from .global_variables import EXTENSION_DESCRIPTION, EXTENSION_TITLE
from .ui_builder import UIBuilder
import rclpy
from geometry_msgs.msg import Quaternion, Pose, Point
from hunav_msgs.srv import ComputeAgents
from hunav_msgs.msg import Agent, Agents, AgentBehavior
from std_msgs.msg import Header
from omni.isaac.dynamic_control import _dynamic_control
import subprocess
import os
import signal
import yaml
import omni.physx
import math

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        self._window = ScrollingWindow(title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM)
        self._window.set_visibility_changed_fn(self._on_window)
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}", self._menu_callback, description=f"Add {EXTENSION_TITLE} Extension to UI toolbar")
        self._menu_items = [MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))]
        add_menu_items(self._menu_items, EXTENSION_TITLE)
        self.ui_builder = UIBuilder()
        self.ui_builder.extension_instance = self
        self._physxIFace = omni.physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('isaac_hunav_bridge')
        self.compute_agents_client = self.node.create_client(ComputeAgents, '/compute_agents')
        self.dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self.agents = []
        self.robot = None
        self.agent_initial_states = []
        self.animationDict=[]
        self.hunavsim_processes=[]

    def initializeHunavNodes(self):
        process_1 = subprocess.Popen(["ros2", "run", "hunav_evaluator", "hunav_evaluator_node"], preexec_fn=os.setsid)
        process_2 = subprocess.Popen(["ros2", "run", "hunav_agent_manager", "hunav_agent_manager"], preexec_fn=os.setsid)#Launch ros2
        self.hunavsim_processes.append(process_1)
        self.hunavsim_processes.append(process_2)
        print(process_1)
        print(process_2)

    def closeHunavNodes(self):
        for process in self.hunavsim_processes:
            print(process)
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        self.hunavsim_processes = []

    def loadConfig(self, file_path):
        yaml_path = os.path.join(os.path.dirname(__file__), file_path)
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    
    def clearSimulation(self):
        self.closeHunavNodes()
        stage = self._usd_context.get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        for prim in world_prim.GetChildren():
            stage.RemovePrim(prim.GetPath())
        self.agents.clear()
        self.robot = None
        self.agent_initial_states.clear()
        self.animationDict.clear()

    def createAnimation(self, animationPath, sourcePath):
        animation=self.stage.DefinePrim(animationPath, "SkelAnimation")
        animation.GetReferences().AddReference(sourcePath)
        return animation
    
    def setAnimation(self, agentPrim, animation):
        skeletonPath = f"{agentPrim.GetPath()}/Root"
        skeletonPrim = self.stage.GetPrimAtPath(skeletonPath)
        bindingApi = UsdSkel.BindingAPI.Apply(skeletonPrim)
        bindingApi.GetAnimationSourceRel().SetTargets(animation)

    def initializeAgents(self):
        self.stage = self._usd_context.get_stage()
        agentConfigs = self.config['hunav_loader']['ros__parameters']['agents']
        rotationVectorXCorrection = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
        rotationQuaternionXCorrection = Gf.Quatf(rotationVectorXCorrection)

        #Crear Animación Walk Loop 1
        anim_walk_loop_1=self.createAnimation("/World/Animations/AnimationWalkLoop1", "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd")

        #Crear Animación Idle
        anim_idle=self.createAnimation("/World/Animations/AnimationIdle", "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Animations/stand_idle_loop.skelanim.usd")

        #CAMBIAR POR ANIMACIONES NUEVAS
        self.animationDict = {
        0: anim_idle.GetPath(),
        1: anim_walk_loop_1.GetPath(),
        2: anim_walk_loop_1.GetPath(),
        3: anim_walk_loop_1.GetPath(),
        4: anim_walk_loop_1.GetPath(),
        5: anim_walk_loop_1.GetPath(),
        6: anim_walk_loop_1.GetPath(),
        #REGULAR=1, IMPASSIVE=2, SURPRISED=3, SCARED=4, CURIOUS=5, THREATENING=6
        }

        for agent_name in agentConfigs:
            agentConfig = self.config['hunav_loader']['ros__parameters'][agent_name]
            asset_path = agentConfig.get('asset_path', "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/biped_demo/biped_demo_meters.usd")
            model = agentConfig.get('model', 'default_model')
            initialPose = self.config['hunav_loader']['ros__parameters'][agent_name]['init_pose']
            initialPosition = Gf.Vec3d(initialPose['x'], initialPose['y'], initialPose['z'])
            initialRotation = Gf.Quatf(1, 0, 0, 0) * rotationQuaternionXCorrection
            self.initializeAgent(self.stage, agent_name, asset_path, model, initialPosition, initialRotation)

            self.agent_initial_states.append({"position": initialPosition, "orientation": initialRotation})
        self.initializeRobot(self.stage)

    def resetAgentStates(self):
        for agent, initial_state in zip(self.agents, self.agent_initial_states):
            agent.GetAttribute("xformOp:translate").Set(initial_state["position"])
            agent.GetAttribute("xformOp:orient").Set(initial_state["orientation"])

    def initializeAgent(self, stage, name, asset_path, model, initialPosition, initialRotation):
        agentPath = f"/World/{name}"
        agent = stage.DefinePrim(agentPath, "SkelRoot")
        agent.GetReferences().AddReference(asset_path)
        UsdPhysics.RigidBodyAPI.Apply(agent)
        PhysxSchema.PhysxRigidBodyAPI.Apply(agent)
        xform = UsdGeom.Xformable(agent)
        xform.AddOrientOp()
        agent.GetAttribute("xformOp:orient").Set(initialRotation)
        agent.GetAttribute("xformOp:translate").Set(initialPosition)
        agent.GetAttribute("physxRigidBody:disableGravity").Set(True)
        self.agents.append(agent)
        self.setAnimation(agent, [self.animationDict.get(0)])

    def initializeRobot(self, stage):
        agentPath="/World/robot"
        asset_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
        robot=stage.DefinePrim(agentPath, "Xform")
        robot.GetReferences().AddReference(asset_path)
        UsdPhysics.RigidBodyAPI.Apply(robot)
        PhysxSchema.PhysxRigidBodyAPI.Apply(robot)
        xform=UsdGeom.Xformable(robot)
        robot.GetAttribute("physxRigidBody:disableGravity").Set(True)
        self.robot=robot

    def getForwardVector(self, quat):
        forward_vector=self.quaternionToForward(quat)
        normalized_forward=self.normalizeVector(forward_vector)
        return normalized_forward
        
    def quaternionToForward(self, quat):
        rotationVectorXCorrection=Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat()
        rotationQuaternionXCorrection=Gf.Quatf(rotationVectorXCorrection)
        quat=quat*rotationQuaternionXCorrection
        x=2*(quat.GetImaginary()[0]*quat.GetImaginary()[1]+quat.GetReal()*quat.GetImaginary()[2])
        y=-(1-2*(quat.GetImaginary()[0]**2+quat.GetImaginary()[2]**2))
        z=2*(quat.GetImaginary()[0]*quat.GetImaginary()[2]-quat.GetReal()*quat.GetImaginary()[1])
        return Gf.Vec3f(x, y, z)
        
    def normalizeVector(self, vector):
        denominador=(vector[0]**2+vector[1]**2+vector[2]**2)**0.5
        if denominador==0:
            return Gf.Vec3f(0, 0, 0)
        return Gf.Vec3f(vector[0]/denominador, vector[1]/denominador, vector[2]/denominador)
    
    def generateLasers(self, numLasers):
        directions = []
        for i in range(numLasers):
            angleRad = math.radians(i * (360.0 / numLasers))
            x = math.cos(angleRad)
            y = math.sin(angleRad)
            directions.append(Gf.Vec3f(x, y, 0))
        return directions

    def getClosestObstacle(self, agent_position, rot, maxDistance):
        directions = self.generateLasers(90)
        closest_hits = []
        for direction in directions:
            hit = omni.physx.get_physx_scene_query_interface().raycast_closest(agent_position, direction, maxDistance)
            if(hit["hit"]):
                closest_hits.append((hit["distance"], hit["position"]))
            else:
                closest_hits.append((maxDistance, (float(10000),float(10000),float(10000))))
        return closest_hits

    def sendAgentsMsg(self):
        agentsMsg = Agents()
        agentsMsg.header = Header()
        agentsMsg.header.stamp.sec = self.node.get_clock().now().to_msg().sec
        agentsMsg.header.stamp.nanosec = self.node.get_clock().now().to_msg().nanosec
        agentsMsg.header.frame_id = "world"
        robotMsg = self.createRobotMsg()
        for index, agent in enumerate(self.agents):
            agentMsg = self.createAgentMsg(agent, index)
            agentsMsg.agents.append(agentMsg)

        if not self.compute_agents_client.wait_for_service(timeout_sec=2.0):
            print("El servicio ComputeAgents no responde")
            return

        self.computeAgents(agentsMsg, robotMsg)

    def createRobotMsg(self):
        robotMsg = Agent()
        robotMsg.id = 0
        robotMsg.type = Agent.ROBOT
        robotMsg.skin = 1
        robotMsg.name = "Robot"
        robotMsg.group_id = 0
        robotMsg.radius = 0.5
        robotMsg.desired_velocity = 1.0
        robotMsg.linear_vel = 1.0
        robotMsg.angular_vel = 0.5
        pos = self.robot.GetAttribute("xformOp:translate").Get()
        rot = self.robot.GetAttribute("xformOp:orient").Get()
        rot_w = rot.GetReal()
        rot_x, rot_y, rot_z = rot.GetImaginary()
        robotMsg.position.position.x = float(pos[0])
        robotMsg.position.position.y = float(pos[1])
        robotMsg.position.position.z = float(pos[2])
        robotMsg.position.orientation.x = float(rot_x)
        robotMsg.position.orientation.y = float(rot_y)
        robotMsg.position.orientation.z = float(rot_z)
        robotMsg.position.orientation.w = float(rot_w)
        robotMsg.yaw = float(rot_z)
        linear=self.robot.GetAttribute("physics:velocity").Get()
        angular = self.robot.GetAttribute("physics:angularVelocity").Get()
        robotMsg.velocity.linear.x = float(linear[0])
        robotMsg.velocity.linear.y = float(linear[1])
        robotMsg.velocity.linear.z = float(linear[2])
        robotMsg.velocity.angular.x = float(angular[0])
        robotMsg.velocity.angular.y = float(angular[1])
        robotMsg.velocity.angular.z = float(angular[2])
        robotMsg.cyclic_goals = True
        robotMsg.goal_radius = 0.5
        robotMsg.closest_obs = []
        return robotMsg

    def createAgentMsg(self, agent, index):
        agent_ref = self.config['hunav_loader']['ros__parameters']['agents'][index]
        agentConfig=self.config['hunav_loader']['ros__parameters'][agent_ref]
        agentMsg = Agent()
        agentMsg.id = int(agentConfig['id'])
        agentMsg.type = Agent.PERSON
        agentMsg.skin = agentConfig['skin']
        agentMsg.name = f"Agent{index + 1}"
        agentMsg.group_id = int(agentConfig['group_id'])
        agentMsg.radius = float(agentConfig['radius'])
        agentMsg.desired_velocity = float(agentConfig['max_vel'])
        agentMsg.linear_vel = 1.0
        agentMsg.angular_vel = 0.5
        pos = agent.GetAttribute("xformOp:translate").Get()
        rot = agent.GetAttribute("xformOp:orient").Get()
        rot_w = rot.GetReal()
        rot_x, rot_y, rot_z = rot.GetImaginary()
        agentMsg.position.position.x = float(pos[0])
        agentMsg.position.position.y = float(pos[1])
        agentMsg.position.position.z = float(pos[2])
        agentMsg.position.orientation.x = float(rot_x)
        agentMsg.position.orientation.y = float(rot_y)
        agentMsg.position.orientation.z = float(rot_z)
        agentMsg.position.orientation.w = float(rot_w)
        agentMsg.yaw = float(rot_z)
        linear = agent.GetAttribute("physics:velocity").Get()
        angular = agent.GetAttribute("physics:angularVelocity").Get()
        agentMsg.velocity.linear.x = float(linear[0])
        agentMsg.velocity.linear.y = float(linear[1])
        agentMsg.velocity.linear.z = float(linear[2])
        agentMsg.velocity.angular.x = float(angular[0])
        agentMsg.velocity.angular.y = float(angular[1])
        agentMsg.velocity.angular.z = float(angular[2])
        agentMsg.cyclic_goals=agentConfig['cyclic_goals']
        agentMsg.goal_radius = float(agentConfig['goal_radius'])
        goals = []
        for goal in agentConfig['goals']:
            goal_config = agentConfig[goal]
            goalPosition = Pose(position=Point(x=goal_config['x'], y=goal_config['y'], z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            goals.append(goalPosition)
        agentMsg.goals = goals
        beh_once=agentConfig['behavior']['once']
        agentMsg.behavior = AgentBehavior(
            type=int(agentConfig['behavior']['type']),
            state=1,
            configuration=int(agentConfig['behavior']['configuration']),
            duration=float(agentConfig['behavior']['duration']),
            once=beh_once,
            vel=float(agentConfig['behavior']['vel']),
            dist=float(agentConfig['behavior']['dist']),
            social_force_factor=float(agentConfig['behavior']['social_force_factor']),
            goal_force_factor=float(agentConfig['behavior']['goal_force_factor']),
            obstacle_force_factor=float(agentConfig['behavior']['obstacle_force_factor']),
            other_force_factor=float(agentConfig['behavior']['other_force_factor'])
        )
        maxDistance = 3.0
        agentMsg.closest_obs=[]
        hits=self.getClosestObstacle(pos, rot, maxDistance)
        for hit in hits:
            agentMsg.closest_obs.append(Point(x=float(hit[1][0]), y=float(hit[1][1]), z=float(hit[1][2])))
        return agentMsg

    def computeAgents(self, agentsMsg, robotMsg):
        try:
            request = ComputeAgents.Request()
            request.current_agents = agentsMsg
            request.robot = robotMsg
            future = self.compute_agents_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            if future.done():
                response = future.result()
                if response is None:
                    print("Respuesta no recibida del servicio")
                else:
                    self.updateAgents(response.updated_agents)
                return response
            else:
                print("La llamada al servicio no completada")
                return None
        except Exception as e:
            print(f"Error al llamar al servicio: {e}")
            return None

    def updateAgents(self, updatedAgents):
        rotationVectorXCorrection = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
        rotationQuaternionXCorrection = Gf.Quatf(rotationVectorXCorrection)
        rotationVectorZCorrection = Gf.Rotation(Gf.Vec3d(0, 1, 0), 90).GetQuat()
        rotationQuaternionZCorrection = Gf.Quatf(rotationVectorZCorrection)
        for updatedAgent in updatedAgents.agents:
            agentPrim = self.agents[updatedAgent.id - 1]

            # POSICION
            updatedPosition = Gf.Vec3d(updatedAgent.position.position.x, updatedAgent.position.position.y, updatedAgent.position.position.z)
            agentPrim.GetAttribute("xformOp:translate").Set(updatedPosition)

            # ROTACION
            new_rot = Gf.Quatf(updatedAgent.position.orientation.w, updatedAgent.position.orientation.x, updatedAgent.position.orientation.y, updatedAgent.position.orientation.z)
            agentPrim.GetAttribute("xformOp:orient").Set(new_rot * rotationQuaternionXCorrection * rotationQuaternionZCorrection)  # Combinar las rotaciones

            # LINEAL
            new_linear = Gf.Vec3d(updatedAgent.velocity.linear.x, updatedAgent.velocity.linear.y, updatedAgent.velocity.linear.z)
            agentPrim.GetAttribute("physics:velocity").Set(new_linear)

            # ANGULAR
            new_angular = Gf.Vec3d(updatedAgent.velocity.angular.x, updatedAgent.velocity.angular.y, updatedAgent.velocity.angular.z)
            agentPrim.GetAttribute("physics:angularVelocity").Set(new_angular)

            #isIddle = (updatedAgent.velocity.linear.x == 0 and updatedAgent.velocity.linear.y == 0 and updatedAgent.velocity.linear.z == 0)
            isIddle = max([abs(updatedAgent.velocity.linear.x),abs(updatedAgent.velocity.linear.y), abs(updatedAgent.velocity.linear.z)])<0.02
            if isIddle:
                self.setAnimation(agentPrim, [self.animationDict.get(0)])
            else:
                self.setAnimation(agentPrim, [self.animationDict.get(updatedAgent.behavior.type)])

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")
        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()
        self.node.destroy_node()
        rclpy.shutdown()
        self.closeHunavNodes()

    def _on_window(self, visible):
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)
            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()
        async def dock_window():
            await omni.kit.app.get_app().next_update_async()
            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window
            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()
        self._task = asyncio.ensure_future(dock_window())

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None
            self.resetAgentStates()
        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        self.sendAgentsMsg()
        self.ui_builder.on_physics_step(step)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        self.ui_builder.build_ui()
