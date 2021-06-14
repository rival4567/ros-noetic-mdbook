# XACRO

Xacro is an XML macro language

With Xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions. Documentation can be found in the wiki: [ROS Wiki](http://wiki.ros.org/xacro)

The xacro program runs all of the macros and outputs the result. Typical usage looks something like this:

```xml
xacro --inorder model.xacro > model.urdf
```

On ROS distros melodic and later, you should omit the {--inorder} argument.

You can also automatically generate the urdf in a launch file. This is convenient because it stays up to date and doesn’t use up hard drive space. However, it does take time to generate, so be aware that your launch file might take longer to start up.