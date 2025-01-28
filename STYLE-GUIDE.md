# Repository Naming Conventions

## Folders and Branches
When creating folders/packages in the source directory, all of your folder names should be in **lowercase**, and with **no spaces**. This convention should also be followed for names of branches on this repository.

### Example
Instead of naming a folder or branch "IntakeShooter", name it "intakeshooter".

## Classes
**All classes** should be written in **Pascal Case**. Which means that the first letter of each word is uppercase, with no dashes or underscores.

### Example
"camelCase.java" &#8594; "PascalCase.java"  

## Subsystems

**IO interfaces** are usually named using the name of the subsystem, appended with "IO". Other names can be used when it makes sense, but for most cases, stick to the subsystem name.

**IO implementations** (the classes that implement your IO interface) should be named the same as the its interface, but with its own name at the end.

**IO inputs** (the inputs object) should be named using the name of the subsystem, appended with "Inputs".

### Note
**For any confusion about the AdvantageKit structure of subsystems, ask, or refer to the documentation [here](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces), and the 2025 kitbot example [here](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/kitbot_2025/src/main/java/frc/robot).**

### Example
Let's take a subsystem named **Arm**.

**The subsystem class** would be named "Arm".

**The IO interface** would be named "ArmIO".

**The IO implementations** would be named "ArmIOHardware", or "ArmIOSim".

**The IO inputs** object would be named "ArmInputs".
