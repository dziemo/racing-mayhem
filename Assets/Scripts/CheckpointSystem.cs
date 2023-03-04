using PathCreation;
using PathCreation.Examples;
using UnityEngine;

public class CheckpointSystem : MonoBehaviour
{
    [SerializeField]
    private PathCreator pathCreator;
    [SerializeField]
    private RoadMeshCreator roadMeshCreator;

    [SerializeField]
    [Min(2)]
    private int checkpointsCount;

    [SerializeField]
    private GameObject checkpointPrefab;
    [SerializeField]
    private Transform checkpointsParent;

    private void Awake()
    {
        CreateCheckpoints();
    }

    private void CreateCheckpoints()
    {
        for (int i = 0; i < checkpointsCount; i++)
        {
            float time = i / (float)checkpointsCount;
            Vector3 position = pathCreator.path.GetPointAtTime(time);
            Vector3 direction = pathCreator.path.GetDirection(time);
            Instantiate(checkpointPrefab, position, Quaternion.LookRotation(direction, Vector3.up), checkpointsParent);
        }
    }
}
