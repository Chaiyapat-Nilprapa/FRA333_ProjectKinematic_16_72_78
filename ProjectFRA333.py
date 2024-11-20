# Corrected Block Diagram Text with Feedforward Control Output linked to Hydraulic System directly

corrected_diagram_text = """
Updated Block Diagram: Integrated System with All Blocks (Corrected)

+-------------------------+
|       Input             |
|-------------------------|
| q (Position)            |
| q_dot (Velocity)        |
| q_dotdot (Acceleration) |
+-------------------------+
            |
            v
+-------------------------+
|   Trajectory Planning   |
|-------------------------|
|   1. Path Generation    |
|   2. Velocity Profile    |
|   3. Acceleration Profile|
|   4. Time Parameterization|
|-------------------------|
| Output:                 |
|   - q (Position)        |
|   - q_dot (Velocity)    |
|   - q_dotdot (Acceleration)|
+-------------------------+
            |
            v
+-------------------------+      +---------------------------+
|  Forward Kinematics    |      |    Feedforward Control     |
|-------------------------|      |---------------------------|
|  1. Calculates Joint    |      |  1. Velocity Feedforward  |
|     Angles for Position |      |  2. Acceleration Feedforward|
|                         |      |  3. Gain Adjustments       |
|-------------------------|      |---------------------------|
| Output:                 |      | Output:                   |
|   - Joint Angles (q)    |      |   - Force Command         |
+-------------------------+      +---------------------------+
            |                                 |
            v                                 |
+-----------------------------------------------+
|                PID Controller                 |
|-----------------------------------------------|
|   1. Compares Position (q)                    |
|   2. Calculates Error                         |
|   3. Adjusts Control Signal                   |
|-----------------------------------------------|
| Output: Control Signal (Force Command)        |
+-----------------------------------------------+
            |
            v
+-----------------------------------------------+
|              Hydraulic System                 |
|-----------------------------------------------|
|   1. Pressure Control                         |
|   2. Flow Control                             |
|   3. Actuators (Hydraulic Cylinders)          |
|   4. Feedback Sensors                         |
|-----------------------------------------------|
| Output:                                       |
|   - Actual Position                           |
|   - Actual Velocity                           |
|   - Force/Pressure Applied                    |
+-----------------------------------------------+
"""

# Create corrected PDF with the adjusted block diagram text
corrected_pdf_path = "/mnt/data/Corrected_Updated_Block_Diagram_Integrated_System.pdf"
with PdfPages(corrected_pdf_path) as pdf:
    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.text(0.5, 0.5, corrected_diagram_text, ha='center', va='center', fontsize=10, family='monospace')
    ax.axis("off")
    pdf.savefig(fig)
    plt.close(fig)

corrected_pdf_path
