# xacro 101

## Source SDF
```xml title="pure sdf"
<?xml version="1.0" ?>
<sdf version="1.5">
    <model name="camera">
        <pose>0 0 0.05 0 1.57 0</pose>
        <link name="link">
            <inertial>
                <mass>0.1</mass>
                <inertia>
                    <ixx>0.000166667</ixx>
                    <iyy>0.000166667</iyy>
                    <izz>0.000166667</izz>
                </inertia>
            </inertial>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.1</size>
                    </box>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.1</size>
                    </box>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
```

## ns
- Add xmlns to root tag (sdf)

```xml
<sdf version="1.5"
    xmlns:xacro="http://www.ros.org/wiki/xacro">
```
## Property
```xml
<xacro:property name="side" value="0.1" />
<xacro:property name="mass" value="0.1" />
```

### usage
```xml
<box>
    <size>${side} ${side} ${side}</size>
</box>
```

### first run
- Rename demo.sdf to demo.sdf.xacro
- Run xacro command from xacro package

```bash
ros2 run xacro xacro demo.sdf.xacro -o demo.sdf
#
# -o: output file
```

---

## Include
```xml
<xacro:include filename="demo.prop.xacro" />
<xacro:include filename="demo.macro.xacro" />
```

!!! Note
    using with find command
    `$(find wasp_pkg)/model/demo/demo.macro.xacro`
    look for file in `<ws>/install/wasp_pkg/share/wasp_pkg/model/demo/demo.macro.xacro`

---

## Macro
```xml
<xacro:macro name="box_inertia" params="m x y z">
    <inertia 
        ixx="${m*(y*y+z*z)/12}" ixy = "0" ixz = "0" 
        iyy="${m*(x*x+z*z)/12}" iyz = "0" 
        izz="${m*(x*x+z*z)/12}" />
</xacro:macro>
```

### usage
```xml
<inertial>
    <mass>${mass}</mass>
    <xacro:box_inertia m="${mass}" x="${side}" y="${side}" z="${side}"/>
</inertial>
```

## so far
implement  
- include  
- property  
- macro  


```xml title="demo.sdf.xacro" linenums="1" hl_lines="4 5 12 17 24"
<?xml version="1.0" ?>
<sdf version="1.5"
    xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="demo.prop.xacro" />
    <xacro:include filename="demo.macro.xacro" />
    
    <model name="camera">
        <pose>0 0 0.05 0 1.57 0</pose>
        <link name="link">
            <inertial>
                <mass>${mass}</mass>
                <xacro:box_inertia m="${mass}" x="${side}" y="${side}" z="${side}"/>
            </inertial>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>${side} ${side} ${side}</size>
                    </box>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <box>
                        <size>${side} ${side} ${side}</size>
                    </box>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
```

```xml title="demo.prop.xacro"
<?xml version="1.0"?>
<root xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:property name="side" value="0.1" />
    <xacro:property name="mass" value="0.1" />
</root>
```

```xml title="demo.macro.xacro"
<?xml version="1.0"?>
<root xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="box_inertia" params="m x y z">
        <inertia ixx="${m*(y*y+z*z)/12}" ixy = "0" ixz = "0" iyy="${m*(x*x+z*z)/12}" iyz = "0" izz="${m*(x*x+z*z)/12}" />
    </xacro:macro>
</root>
```

---
## args
- Get argument from command line

```xml title="arg"
<xacro:arg name="type" default="0"/>
<xacro:if value="$(arg type)">
    <xacro:property name="mass" value="0.2" />
</xacro:if>
```

### usage
```
ros2 run xacro xacro demo.sdf.xacro -o demo.sdf type:=true
ros2 run xacro xacro demo.sdf.xacro -o demo.sdf type:=false
```

## Conditions
```xml title="property"
<xacro:property name="btype" value="big" />
<xacro:if value="${btype=='big'}">
    <xacro:property name="mass" value="0.2" />
</xacro:if>
```

```xml title="arg"
<xacro:arg name="type" default="0"/>
<xacro:if value="$(arg type)">
    <xacro:property name="mass" value="0.2" />
</xacro:if>
```
