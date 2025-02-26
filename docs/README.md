# GaTORS: A Game-Theoretic Tool for Optimal Robot Selection and Design in Surface Coverage Applications

#### Steven Swanbeck, Daniel Meza, Jared Rosenbaum, David Fridovich-Keil, and Mitch Pryor

{% include youtube.html id="GM4DKKhbErg" %}

### About
As the number of commercially-available robots
increases, users face the challenge of evaluating many options
to identify the optimal system for their needs. This market
saturation also compels providers to ensure new systems are
competitive with or superior to existing robots to increase
economic viability. The need for evaluation extends to multi-
robot teams collaborating toward shared objectives, where
understanding individual contributions to overall team per-
formance is complex but necessary. One specific application
domain for robot platform selection in industry is autonomous
surface coverage, which includes tasks such as painting, clean-
ing, and surveying in industrial facilities. To assist in the
design and selection of robotic systems for surface coverage
applications, we introduce GaTORS, a novel tool that frames
the surface coverage task allocation process as a collaborative
general-sum discrete-time game. By parameterizing robots with
a set of common constraints, this tool enables performance
evaluation of existing and potential future robotic systems.
GaTORS is evaluated in a case study of surface coverage for
corrosion mitigation in an industrial refinery, with experiments
demonstrating its utility in selecting existing robotic platforms
best suited to complete the specific coverage task. These
experiments also highlight GaTORS’ potential to inform the
design of new systems that can efficiently accomplish assigned
tasks within practical time and cost constraints. Due to its
flexibility, GaTORS can be easily adapted to provide similar
insights for other types of robots in different environments and
surface coverage applications.

### Methods
GaTORS frames the problem of multi-robot task allocation as a collaborative game parameterized by environment and robot-specific constraints. It is designed to be flexible, allowing custom environments and robots to be implemented to provide robot selection and design guidance based on simulation results in specific environments.

As an example, GaTORS is used to inform robot selection and design choices for peroforming surface coverage for repair of corroded material in an industrial refinery. Material to be colored is marked in pink.

![Map](/assets/map.png)

Heterogeneous teams of drone, quadruped, and gantry systems are evaluated for collaboratively completing the repair task.

![Full](/assets/full.svg)

Based on simulation results, team compositions capable of completing the task under reasonable time and cost considerations are suggested.

![TeamComposition](/assets/team_composition.svg)

Simulation results suggest the drone system is weaker than both the quadruped and gantry systems for this task, so GaTORS is used to perform a parameter sweep over possible drone configurations to set peroformance targtes that would make it competitive.

![RobotDesign](/assets/drone_design.svg)

For details, readers are referred to the paper.
