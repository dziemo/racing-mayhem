using System.Collections.Generic;
using UnityEngine;

namespace RacingMayhem
{
    public class TestCar : MonoBehaviour
    {
        [SerializeField]
        private Rigidbody rb;
        [SerializeField]
        private float carTopSpeed;
        [SerializeField]
        private AnimationCurve powerCurve;
        [SerializeField]
        private float reverseSpeed;
        [SerializeField]
        private Transform centerOfMass;
        [SerializeField]
        private List<Transform> raycastTransforms = new List<Transform>();

        private ICarInput carInput;

        private void Awake()
        {
            rb.centerOfMass = centerOfMass.localPosition;
            rb.ResetInertiaTensor();
            if (TryGetComponent(out ICarInput carInput))
            {
                this.carInput = carInput;
            }
        }

        private void FixedUpdate()
        {
            foreach (Transform wheel in raycastTransforms)
            {
                ForwardForce(wheel);
            }
        }

        private void ForwardForce(Transform tireTransform)
        {
            Vector3 accelDir = tireTransform.forward;

            int notTouchingWheelsFactor = 1;

            if (carInput.VerticalInput > 0)
            {
                float carSpeed = Vector3.Dot(transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * carTopSpeed * notTouchingWheelsFactor;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
            else if (carInput.VerticalInput < 0)
            {
                float carSpeed = Vector3.Dot(-transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / reverseSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * reverseSpeed * notTouchingWheelsFactor;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
        }
    }
}
