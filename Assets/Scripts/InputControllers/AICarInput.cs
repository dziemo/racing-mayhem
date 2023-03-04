using System.Collections.Generic;
using UnityEngine;

namespace RacingMayhem
{
    public class AICarInput : MonoBehaviour, ICarInput
    {
        [SerializeField]
        private List<Transform> path = new List<Transform>();

        public float HorizontalInput => horizontalInput;

        public float VerticalInput => verticalInput;

        [SerializeField]
        private float nextPathPointDistanceTreshold = 0.15f;

        private int currentPathPointIndex = 0;
        private float horizontalInput;
        private float verticalInput = 1;

        private void Update()
        {
            SetSteeringValues();
        }

        private void SetSteeringValues()
        {
            Vector3 dir = (GetCurrentWaypointPosition() - transform.position).normalized;
            horizontalInput = Vector3.Dot(transform.right, dir);
        }

        private void LateUpdate()
        {
            DistanceCheck();
        }

        private void DistanceCheck()
        {
            Vector3 playerPos = transform.position;
            playerPos.y = 0;
            Vector3 waypointPos = GetCurrentWaypointPosition();
            waypointPos.y = 0;
            if (Vector3.Distance(playerPos, waypointPos) < nextPathPointDistanceTreshold)
            {
                if (++currentPathPointIndex >= path.Count)
                {
                    currentPathPointIndex = 0;
                }
            }
        }

        private Vector3 GetCurrentWaypointPosition()
        {
            return path[currentPathPointIndex].position;
        }
    }
}