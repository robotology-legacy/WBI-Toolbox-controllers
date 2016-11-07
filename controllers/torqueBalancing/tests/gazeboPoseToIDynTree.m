function [iDynTreeTransform] = gazeboPoseToIDynTree(gazeboPose)
%gazeboPoseToIDynTree Convert a pose in Gazebo format to iDynTree

pos = iDynTree.Position();
pos.fromMatlab(gazeboPose(1:3));

rot = iDynTree.Rotation.RPY(gazeboPose(4),gazeboPose(5),gazeboPose(6));
rot.toMatlab()

iDynTreeTransform = iDynTree.Transform(rot,pos);

end

