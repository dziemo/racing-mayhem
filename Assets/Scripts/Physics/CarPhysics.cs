using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SocialPlatforms;

namespace RacingMayhem
{
    public enum AccelerationType { Forward, Rear, Full }

    public class CarPhysics : MonoBehaviour
    {
        [SerializeField]
        private Rigidbody rb;
        [SerializeField]
        private Transform centerOfMass;
        [SerializeField]
        private LayerMask collisionMask;
        [Header("Acceleration")]
        [SerializeField]
        private float carTopSpeed;
        [SerializeField]
        private AnimationCurve powerCurve;
        [SerializeField]
        private float brakeForce;
        [SerializeField]
        [Range(0.01f, 0.99f)]
        private float rollingFrictionFactor;
        [SerializeField]
        private float brakeTreshold;
        [SerializeField]
        private float reverseSpeed;
        [SerializeField]
        private AccelerationType accelerationType;
        [Header("Steering")]
        [SerializeField]
        [Range(0f, 1f)]
        private float tireGripFactor;
        [SerializeField]
        private float tireMass;
        [SerializeField]
        private float maxTireRotation;
        [SerializeField]
        private float tireRotationEase;

        [Header("Suspension")]
        [SerializeField]
        private List<Transform> raycastTransforms = new List<Transform>();
        [SerializeField]
        private List<Transform> tireVisuals = new List<Transform>();
        [SerializeField]
        private float raycastLength;
        [SerializeField]
        private float suspensionRestDist;
        [SerializeField]
        private float springStrength;
        [SerializeField]
        private float springDamper;
        [SerializeField]
        private float suspensionSmoothTime;
        [SerializeField]
        private int supportingRayChecks = 2;
        [SerializeField]
        private float rayCheckRange = 30;

        private ICarInput carInput;
        private Vector3 suspensionSmoothingVelocity;

        private void Awake()
        {
            rb.centerOfMass = centerOfMass.position;

            if (TryGetComponent(out ICarInput carInput))
            {
                this.carInput = carInput;
            }
        }

        private void FixedUpdate()
        {
            ApplyTireForces();
        }

        private void Brake(float brakeForce)
        {
            for (int i = 0; i < raycastTransforms.Count; i++)
            {
                Transform tireTransform = raycastTransforms[i];
                Vector3 movingDir = tireTransform.forward;
                Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
                float movingVel = Vector3.Dot(movingDir, tireWorldVel);
                float desiredVelChange = -movingVel * brakeForce;
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
                Vector3 force = desiredAccel * tireMass * movingDir;
                rb.AddForceAtPosition(force, tireTransform.position);

                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
        }

        private void ApplyTireForces()
        {
            for (int i = 0; i < raycastTransforms.Count; i++)
            {
                Transform tireTransform = raycastTransforms[i];
                RaycastHit hitInfo = GroundCheck(tireTransform, supportingRayChecks, rayCheckRange);

                if (i < 2)
                {
                    tireTransform.localRotation = Quaternion.Lerp(tireTransform.localRotation, Quaternion.Euler(new Vector3(0, maxTireRotation * carInput.HorizontalInput)), tireRotationEase * (1 / (1 + rb.velocity.magnitude * Mathf.Abs(carInput.HorizontalInput)) * Time.fixedDeltaTime));
                }

                if (hitInfo.collider != null)
                {
                    SuspensionForce(tireTransform, hitInfo, tireVisuals[i]);
                    if (accelerationType == AccelerationType.Forward && i < 2)
                    {
                        ForwardForce(tireTransform);
                    }
                    else if (accelerationType == AccelerationType.Rear && i >= 2)
                    {
                        ForwardForce(tireTransform);
                    }
                    else
                    {
                        ForwardForce(tireTransform);
                    }

                    SteeringForce(tireTransform);

                }
                else
                {
                    FullyExtendSuspension(tireVisuals[i]);
                }
            }
        }

        private void SteeringForce(Transform tireTransform)
        {
            Vector3 steeringDir = tireTransform.right;
            Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
            float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
            float desiredVelChange = -steeringVel * tireGripFactor;
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
            Vector3 force = desiredAccel * tireMass * steeringDir;
            rb.AddForceAtPosition(force, tireTransform.position);
            Debug.DrawRay(tireTransform.position, force.normalized, Color.red);
        }

        private void SuspensionForce(Transform tireTransform, RaycastHit hitInfo, Transform tireVisual)
        {
            Vector3 springDir = Vector3.up;
            Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
            float offset = suspensionRestDist - hitInfo.distance;
            float velocity = Vector3.Dot(springDir, tireWorldVel);
            float springForce = (offset * springStrength) - (velocity * springDamper);
            Vector3 forceApplied = springDir * springForce;
            rb.AddForceAtPosition(forceApplied, tireTransform.position);
            tireVisual.localPosition = Vector3.SmoothDamp(tireVisual.localPosition, new Vector3(tireVisual.localPosition.x, -hitInfo.distance + 0.3f, 0), ref suspensionSmoothingVelocity, suspensionSmoothTime);

            Debug.DrawRay(tireTransform.position, forceApplied.normalized, Color.green);
        }

        private void FullyExtendSuspension(Transform tireVisual)
        {
            tireVisual.localPosition = Vector3.SmoothDamp(tireVisual.localPosition, new Vector3(tireVisual.localPosition.x, -raycastLength + 0.3f, 0), ref suspensionSmoothingVelocity, suspensionSmoothTime);
        }

        private void ForwardForce(Transform tireTransform)
        {
            Vector3 accelDir = tireTransform.forward;

            if (carInput.VerticalInput > 0)
            {
                float carSpeed = Vector3.Dot(transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * carTopSpeed;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
            else if (carInput.VerticalInput < 0)
            {
                float carSpeed = Vector3.Dot(-transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / reverseSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * reverseSpeed;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
            else
            {
                Brake(brakeForce * rollingFrictionFactor);
            }
        }

        private RaycastHit GroundCheck(Transform tireTransform, int rays, float range)
        {

            if (Physics.Raycast(tireTransform.position, -tireTransform.up, out RaycastHit hitInfo, raycastLength, collisionMask))
            {
                return hitInfo;
            }

            float step = range / rays;
            float angle = -range / 2;
            for (int i = 0; i < rays; i++)
            {
                if (i == rays / 2)
                {
                    angle += step;
                }
                Vector3 supportingRayDirection = Quaternion.Euler(angle + step * i, 0, 0) * -tireTransform.up;
                supportingRayDirection = tireTransform.TransformDirection(supportingRayDirection);
                if (Physics.Raycast(tireTransform.position, supportingRayDirection, out RaycastHit newHitInfo, raycastLength, collisionMask))
                {
                    return newHitInfo;
                }
                else
                {
                    Debug.DrawRay(tireTransform.position, supportingRayDirection * raycastLength, Color.magenta);
                }
            }

            return new RaycastHit();
        }
    }
}