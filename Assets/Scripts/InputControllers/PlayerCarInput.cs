using RacingMayhem;
using UnityEngine;

public class PlayerCarInput : MonoBehaviour, ICarInput
{
    public float HorizontalInput => horizontalInput;
    public float VerticalInput => verticalInput;

    private float horizontalInput;
    private float verticalInput;

    private void Update()
    {
        horizontalInput = Input.GetAxisRaw("Horizontal");
        verticalInput = Input.GetAxisRaw("Vertical");
    }
}
