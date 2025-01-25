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

**IO interfaces** should be named using the name of the subsystem, appended with "IO".

**IO implementations** (the classes that implement your IO interface) should be named the same as the its interface, but with its own name at the end.

**IO inputs** (the inputs object) should be named using the name of the subsystem, appended with "Inputs".

### Example
Lets take a subsystem named **Arm**.

**The subsystem class** would be named "Arm".

**The IO interface** would be named "ArmIO".

**The IO implementations** would be named "ArmIOTalonFX", "ArmIOHardware", or "ArmIOSim".

**The IO inputs** object would be named "ArmInputs".