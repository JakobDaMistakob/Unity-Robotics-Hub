using RosMessageTypes.Moveit;
using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class MoverServiceResponseVisualizer : BasicVisualizer<MMoverServiceResponse>
{
    public Color ghostColor;
    public float thickness = 0.01f;
    public float labelSpacing = 0.1f;
    public UrdfRobot forRobot;
    RobotVisualization robotVisualization;

    public override void Start()
    {
        base.Start();
        robotVisualization = new RobotVisualization(forRobot);
    }

    public override void Draw(MMoverServiceResponse message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        int Idx = 1;
        foreach(MRobotTrajectory trajectory in message.trajectories)
        {
            RobotVisualization.JointPlacement[][] jointPlacements = robotVisualization.GetJointPlacements(trajectory.joint_trajectory);
            RobotVisualization.JointPlacement[] finalPose = jointPlacements[jointPlacements.Length - 1];

            robotVisualization.DrawJointPaths(drawing, jointPlacements, color, thickness);
            robotVisualization.DrawGhost(drawing, finalPose, ghostColor);

            drawing.DrawLabel(Idx.ToString(), finalPose[finalPose.Length - 1].position, color, labelSpacing);
            ++Idx;
        }
    }
}
/*{
    public float thickness = 0.01f;
    public UrdfRobot robot;
    Dictionary<string, Tuple<UrdfJoint, Quaternion>> jointsByName = new Dictionary<string, Tuple<UrdfJoint, Quaternion>>();

    public override void Start()
    {
        foreach(UrdfJoint joint in robot.gameObject.GetComponentsInChildren<UrdfJoint>())
        {
            if(!jointsByName.ContainsKey(joint.jointName))
                jointsByName.Add(joint.jointName, new Tuple<UrdfJoint, Quaternion>(joint, joint.transform.localRotation));
        }
        base.Start();
    }

    public override void Draw(MMoverServiceResponse message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        List<Vector3> endEffectorPoses = new List<Vector3>();
        for(int tIdx = 0; tIdx < message.trajectories.Length; ++tIdx)
        {
            MRobotTrajectory trajectory = message.trajectories[tIdx];
            string[] jointNames = trajectory.joint_trajectory.joint_names;

            Color blendedColor = Color.Lerp(Color.blue, color, tIdx / (float)message.trajectories.Length);
            blendedColor.a = 0.2f;
            //Vector3 endEffectorPrevPos = Vector3.zero;
            for (int pIdx = 0; pIdx < trajectory.joint_trajectory.points.Length; ++pIdx)
            {
                MJointTrajectoryPoint point = trajectory.joint_trajectory.points[pIdx];
                Quaternion lastRotation = robot.transform.rotation;
                Vector3 lastWorldPosition = robot.transform.position;
                GameObject lastJoint = robot.gameObject;
                bool isEndPoint = pIdx == trajectory.joint_trajectory.points.Length-1;
                for (int Idx = 0; Idx < point.positions.Length; ++Idx)
                {
                    (UrdfJoint joint, Quaternion baseRotation) = jointsByName[jointNames[Idx]];
                    float rotationDegrees = (float)(point.positions[Idx] * Mathf.Rad2Deg);

                    ArticulationBody body = joint.GetComponent<ArticulationBody>();
                    Quaternion jointRotation = body.anchorRotation * Quaternion.Euler(rotationDegrees, 0, 0) * Quaternion.Inverse(body.anchorRotation);
                    /*
                    ArticulationDrive drive = body.xDrive;
                    drive.target = rotationDegrees;
                    body.xDrive = drive;* /
                    Quaternion localRotation = baseRotation * jointRotation;
                    Vector3 localPosition = lastJoint.transform.InverseTransformPoint(joint.transform.position);
                    Vector3 worldPosition = lastWorldPosition + lastRotation * localPosition;
                        //drawing.DrawLine(lastWorldPosition, worldPosition, blendedColor, thickness);

                    lastWorldPosition = worldPosition;
                    lastRotation *= localRotation;
                    lastJoint = joint.gameObject;

                    if (isEndPoint)
                    {
                        Transform visual = joint.transform.Find("Visuals/unnamed");
                        foreach(MeshFilter mfilter in visual.GetComponentsInChildren<MeshFilter>())
                        {
                            Vector3 localMeshOffset = lastRotation * joint.transform.InverseTransformPoint(mfilter.transform.position);
                            Quaternion localMeshRotation = Quaternion.Inverse(joint.transform.rotation) * mfilter.transform.rotation;
                            drawing.DrawMesh(mfilter.mesh, lastWorldPosition + localMeshOffset, lastRotation * localMeshRotation, Vector3.one, blendedColor);
                        }
                    }
                    //joint.transform.localRotation = localRotation;
                    //body.enabled = false;
                }

                if (pIdx > 0)
                {
                    endEffectorPoses.Add(lastWorldPosition);
                }
                //endEffectorPrevPos = lastWorldPosition;
            }
            //base.Draw(message, meta, color, label, drawing);

            for(int Idx = 1; Idx < endEffectorPoses.Count; ++Idx)
            {
                drawing.DrawLine(
                    endEffectorPoses[Idx - 1],
                    endEffectorPoses[Idx],
                    Color.Lerp(Color.black,
                    Color.green,
                    Idx / (float)(endEffectorPoses.Count)),
                    thickness
                );
            }
        }
    }

    public override Action CreateGUI(MMoverServiceResponse message, MessageMetadata meta, DebugDraw.Drawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
*/