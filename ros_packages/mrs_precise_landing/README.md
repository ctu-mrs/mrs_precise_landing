# Landing Object Controller

## Action Server

The action server's goal has following content:

```
int32 goal
geometry_msgs/point position 
```

where **goal** is 0 for dropping, 1 for nearest static, 2 for nearest moving and 3 for nearest long object. The **position** is used to drop the object on a particular coordinates.
