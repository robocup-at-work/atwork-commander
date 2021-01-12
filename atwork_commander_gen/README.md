# atwork\_commander\_gen

Enable dynamic task generation according to specified task types of the [@Work
RuleBook](https://github.com/robocup-at-work/rulebook). 

## Generators

There are currently two generators implemented:

* Simple: Simple task generator to be used as basis for own creations. __Not Rule-Book compatible__
* JurekGen: Generator created by Jurek Rostalsky from the robOTTO Team with additional integration code. Aims for
  Rule-Book compatibility, but because of the integration code the software architecture is rather messed up.

Generators can be configured through the launch parameter _generator_

## Configuration

Uses simple ROS parameter server-based specification of the task types and generates
tasks accordingly.

### Arena Configuration

### Task Configuration

## Architecture


