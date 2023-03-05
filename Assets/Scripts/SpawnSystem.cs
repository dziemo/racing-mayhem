using PathCreation;
using UnityEngine;

namespace RacingMayhem
{
    public class SpawnSystem : MonoBehaviour
    {
        [SerializeField]
        private PathCreator pathCreator;

        [SerializeField]
        private CarPhysics carPrefab;

        private void Awake()
        {
            Vector3 position = pathCreator.path.GetPointAtTime(0.99f);
            Vector3 direction = pathCreator.path.GetDirection(0.99f);

            CarPhysics car = Instantiate(carPrefab, position, Quaternion.LookRotation(direction, Vector3.up));
            car.Initialize();
        }
    }
}
