# Nav2 Skills

Author: [Michael Terzer](mailto:michael.terzer@fraunhofer.it) - Fraunhofer Italia (2023)

This package provides skill implementations for the navigation2 stack in form of behavior tree nodes.  The nodes get registered and loaded by the plugin `nav2_skills::SkillLibrary`, which inherits from the [behavior_tree_stack](https://fhi-git01.fraunhofer.it/behavior-tree/behavior_tree_stack) `bt_skill::SkillLibraryBase` class. 



### Navigate To Pose

The navigate to pose action can be used to send a mobile robot to a desired pose on a planar map. 

The **inputs** of this node are the `action_name` and the pose in form of

 `"[x; y; theta]"`. 

The action call in the behavior tree can be done like:

```xml
<?xml version="1.0"?>
<root BTCPP_format="4"  main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Action ID="NavigateToPose" action_name="navigate_to_pose" 
pose="0.5;0.0;0.0"/>
        </Sequence>
    </BehaviorTree>
</root>

```
