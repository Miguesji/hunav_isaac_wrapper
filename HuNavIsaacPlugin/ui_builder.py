# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.timeline
import omni.ui as ui
from omni.isaac.ui.element_wrappers import CollapsableFrame
from omni.isaac.ui.ui_utils import get_style
from .world_builder import WorldBuilder
import os

class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self.map_loader = WorldBuilder(base_path=os.path.dirname(__file__))

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        pass

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        pass

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        main_panel = CollapsableFrame("Configuration", collapsed=False)

        with main_panel:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                self._init_button = ui.Button("Initialize", clicked_fn=self._on_init_button_clicked)
                self._config_path_field = ui.StringField()
                self._config_path_field.model.set_value("config/agents_1.yaml")
        
        debug_panel = CollapsableFrame("Debug Tools", collapsed=True)

        with debug_panel:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._init_button = ui.Button("Close_Hunav_Nodes", clicked_fn=self.close_nodes)


    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        pass

    def close_nodes(self):
        self.extension_instance.closeHunavNodes()
    
    def _on_init_button_clicked(self):
        config_path = self._config_path_field.model.get_value_as_string()
        self.extension_instance.clearSimulation()
        self.extension_instance.config = self.extension_instance.loadConfig(config_path)
        self.map_loader.load_map(self.extension_instance.config['hunav_loader']['ros__parameters']['map'])
        self.extension_instance.initializeAgents()
        self.extension_instance.initializeHunavNodes()

    def _on_file_selected(self, selected_file):
        if selected_file:
            self._config_path_field.model.set_value(selected_file)
