<h1>FRA333: SIMULATION AND TRAJECTORY PLANNING OF BACKHOE ARM USING FORWARD KINEMATICS AND DYNAMIC ANALYSIS </h1>
<h2>จัดทำโดย</h2>
<p>
1. ชัยภัทร นิลประภา 65340500016
</p>
<p>
2. พิมพ์ณภัทร ปูอินต๊ะ 65340500072
</p>
<p>
3. ชนัญญ์ทิชา โพธิ์พันธ์ 65340500078
</p>

<h2>บทนำ</h2>
<p>
โปรเจคนี้เป็นการพัฒนาโปรแกรมจำลองการเคลื่อนที่ของแขนกลแบคโฮ (Backhoe) 
โดยคำนวณหาตำแหน่งการหมุนของแต่ละมุมข้อต่อ เพื่อให้ได้ตำแหน่งปลาย End Effector ที่ต้องการ 
ของระบบ Open Loop Kinematic Chain ในการจำลองนี้จะอ้างอิงความยาวของ link แต่ละส่วน 
(Boom, Arm, Bucket และ Rotate Base) และมุมของข้อต่อ (Joint Angles) ของแบคโฮที่ใช้งาน 
โดยเฉพาะ Rotate Base ซึ่งเป็นส่วนฐานที่สามารถหมุนได้ 360 องศา 
ทำให้เพิ่มระดับความอิสระ (Degrees of Freedom) จากเดิม 3 DoF เป็น 4 DoF 
เพื่อให้ครอบคลุมทุกมิติของการเคลื่อนที่ของแขนกลแบคโฮ 
การคำนวณหาตำแหน่งการหมุนของแต่ละมุมข้อต่อ จะใช้หลักการ Inverse Kinematics 
ร่วมกับ Forward Kinematics เพื่อสร้างแบบจำลองการเคลื่อนที่ให้ใกล้เคียงความเป็นจริง 
สำหรับการวางแผนเส้นทาง Trajectory Planning นำมาใช้เพื่อวางแผนการเคลื่อนที่ของแขนกลแบคโฮให้ต่อเนื่อง 
โดยกำหนดตำแหน่ง ความเร็ว และความเร่งของแขนกลในแต่ละช่วงเวลา 
ทางผู้จัดได้มีการคำนวณ Dynamic Analysis เพื่อทำการจำลองแรงที่เกิดขึ้นโดยการเพิ่มน้ำหนักบริเวณปลายแขน End Effector 
โดยคำนึงถึงแรงที่กระทำในแต่ละข้อต่อและปลาย End Effector 
เพื่อให้สามารถวิเคราะห์และประเมินผลกระทบจากแรงที่เกิดขึ้นระหว่างการทำงาน
</p>

<h2>วัตถุประสงค์</h2>
<p> 2.1 เพื่อศึกษาการทำงานของระบบ Open Loop Kinematic Chain ในปริภูมิ 3 มิติ สำหรับความยาวของ Link แต่ละส่วน (Boom, Arm, Bucket, Rotate Base) และ มุมการหมุนในข้อต่อ (Joint Angles)</p>

<p> 2.2 เพื่อศึกษาการจำลองแบบ Dynamic ในการทำงานของแขน Backhoe ขณะตักดิน โดยคำนึงถึงแรงที่กระทำในแต่ละข้อต่อ และการวิเคราะห์การเปลี่ยนแปลงของแรงบิดที่เกิดขึ้น</p>

<p> 2.3 เพื่อศึกษาการวางแผนเส้นทางและการควบคุมการเคลื่อนที่ของ Backhoe โดยออกแบบ Trajectory Planning สำหรับการกำหนดเส้นทางและตำแหน่งในการเคลื่อนที่ของแขนกล</p>

<h2>System overview</h2>

![System diagram](Picture/System%20diagram.png)


# Forward Kinematic

## บทนำ
โปรเจคนี้เป็นการคำนวณตำแหน่งปลายมือ (End Effector) ของแขนกล โดยการกำหนดค่ามุมของแต่ละข้อต่อ (Joint Angles) ด้วยหลักการ **Forward Kinematics** ซึ่งใช้หลักการ Denavit-Hartenberg (DH) ในการกำหนดพิกัดเชิงเส้นและเชิงมุมสำหรับแต่ละข้อต่อในระบบ Open Loop Kinematic Chain

พารามิเตอร์ DH ที่ใช้ในการคำนวณและสร้างแบบจำลองการเคลื่อนที่ ประกอบด้วย 4 พารามิเตอร์ต่อข้อต่อ 1 จุด หรือ 1 ลิงค์ ดังนี้:

| ข้อต่อ (Joint) | A<sub>i</sub> | α<sub>i</sub> | D<sub>i</sub>           | Θ<sub>i</sub>          |
|------------------|---------------|---------------|-------------------------|-------------------------|
| 1                | 0             | 0             | D<sub>1</sub>+L<sub>1</sub> | Θ<sub>1</sub>          |
| 2                | A<sub>1</sub> | 90            | 0                       | Θ<sub>2</sub>          |
| 3                | L<sub>2</sub> | 0             | 0                       | 90+Θ<sub>3</sub>       |
| 4                | L<sub>3</sub> | 0             | 0                       | Θ<sub>4</sub>          |
| 5                | L<sub>4</sub> | 0             | 0                       | 0                       |

### คำอธิบาย DH Parameters:
- **Θ<sub>i</sub>** = มุมระหว่างแกน Z<sub>i-1</sub> และ Z<sub>i</sub>
- **D<sub>i</sub>** = ระยะห่างเชิงเส้นระหว่างแกน Z<sub>i-1</sub> และ Z<sub>i</sub> วัดในระนาบ Z<sub>i-1</sub>
- **A<sub>i</sub>** = ระยะห่างเชิงเส้นระหว่างแกน X<sub>i-1</sub> และ X<sub>i</sub> วัดในระนาบ X<sub>i-1</sub>
- **α<sub>i</sub>** = การหมุนรอบแกน X<sub>i-1</sub>
- **L<sub>i</sub>** =  ความยาวของแต่ละลิงค์

---

## การคำนวณเมทริกซ์การแปลง (Transformation Matrix)
เพื่อคำนวณตำแหน่งและการหมุนของปลายแขนกล (End Effector) จะใช้ **เมทริกซ์การแปลง (Transformation Matrix)** ซึ่งคำนวณทีละข้อต่อโดยมีสูตรดังนี้:

![Transformation Matrix](Picture/FK.png)

โดย:
- **A<sub>i</sub>**: เป็นเมทริกซ์การแปลงของข้อต่อ `i` ที่แสดงการแปลงพิกัดจากข้อต่อ `i-1` ไปยังข้อต่อ `i`

เพื่อคำนวณตำแหน่งของปลายแขน (End Effector) เมื่อมีหลายข้อต่อ จะต้องคูณเมทริกซ์การแปลงระหว่างข้อต่อทั้งหมดเข้าด้วยกัน:
T = A<sub>1</sub> × A<sub>2</sub> × A<sub>3</sub> × … × A<sub>n</sub>


โดย:
- **T**: เมทริกซ์การแปลงสุดท้ายที่บ่งบอกถึงตำแหน่งและการหมุนของปลายแขนในระบบพิกัดฐาน (Base Frame)

---

## การแยกตำแหน่งและการหมุน
จากเมทริกซ์การแปลงสุดท้าย (T) สามารถแยกส่วนที่เกี่ยวข้องดังนี้:
1. **ตำแหน่ง (Position)** = ใช้ค่าในแถวที่ 4 คอลัมน์ที่ 1 ถึง 3 ซึ่งก็คือ `[X, Y, Z]`
2. **การหมุน (Rotation)** = ใช้ส่วนของเมทริกซ์การหมุน (Rotation Matrix) ที่อยู่ในมุมซ้ายบน ขนาด 3×3

---
# Inverse Kinematics

การคำนวณ Inverse Kinematics เป็นการหาค่ามุมของแต่ละข้อต่อที่สอดคล้องกับตำแหน่งปลายมือ (End Effector) ที่กำหนด โดยอ้างอิงจาก Forward Kinematics เพื่อให้ได้ค่ามุมที่เหมาะสมสำหรับแต่ละข้อต่อ โดยสามารถคำนวณตำแหน่งของแต่ละข้อต่อได้ดังนี้:

---

สมการการคำนวณมุมข้อต่อ:
- Θ<sub>1</sub> = atan2(p<sub>y</sub>, p<sub>x</sub>)
- Θ<sub>2</sub> = atan2(sinΘ<sub>2</sub>, cosΘ<sub>2</sub>)
- Θ<sub>3</sub> = atan2(sinΘ<sub>3</sub>, cosΘ<sub>3</sub>)
- Θ<sub>4</sub> = atan2(N<sub>Z</sub>, O<sub>Z</sub>) - Θ<sub>2</sub> - Θ<sub>3</sub>

---

โดยที่
- sinΘ<sub>2</sub> = (BC - AD) / (C<sup>2</sup> + D<sup>2</sup>)
- cosΘ<sub>2</sub> = √(1 - sin<sup>2</sup>Θ<sub>2</sub>)

- cosΘ<sub>3</sub> = (A<sup>2</sup> + B<sup>2</sup> - L<sub>2</sub><sup>2</sup> - L<sub>3</sub><sup>2</sup>) / (2 × L<sub>2</sub> × L<sub>3</sub>)
- sinΘ<sub>3</sub> = √(1 - cos<sup>2</sup>Θ<sub>3</sub>)

---

เเละ
- A = p<sub>x</sub> × cosΘ<sub>1</sub> + p<sub>y</sub> × sinΘ<sub>1</sub> - a<sub>1</sub> - L<sub>4</sub>
- B = p<sub>z</sub> - (d<sub>1</sub> + L<sub>1</sub>) - L<sub>4</sub>
- C = L<sub>2</sub> + L<sub>3</sub> × cosΘ<sub>3</sub>
- D = L<sub>3</sub> × sinΘ<sub>3</sub>
  
---
# Dynamic 
## บทนำ

---
# Trajectory Planning
![Trjectory Planning](Picture/Traject.png)

ทางผู้จัดทำได้เลือกใช้ quintic polynomial trajectory เนื่องจากเป็นสมการพหุนามดีกรีที่ 5 ซึ่งสามารถคำนวณตำแหน่ง (q) ความเร็ว (q<sub>d</sub>) และความเร่ง (q<sub>dd</sub>) ของจุดที่ต้องการเคลื่อนที่ได้อย่างราบรื่นในช่วงเวลาที่กำหนด T โดยอ้างอิงจากเงื่อนไขที่กำหนดไว้ในจุดเริ่มต้นและจุดสิ้นสุด

## การทำงานของฟังก์ชัน

### อินพุต (Inputs)
- **qf**: ตำแหน่งของข้อต่อในแต่ละส่วน
- **t**: เวลาปัจจุบัน

### เอาต์พุต (Outputs)
- **q**: ตำแหน่ง ณ เวลาที่กำหนด (t)
- **q<sub>d</sub>**: ความเร็ว ณ เวลาที่กำหนด (t)
- **q<sub>dd</sub>**: ความเร่ง ณ เวลาที่กำหนด (t)

### กำหนดค่าสภาวะเริ่มต้นและสุดท้าย
- **q0** = 0: ตำแหน่งเริ่มต้น
- **q<sub>d</sub>** = 0: ความเร็วเริ่มต้น
- **q<sub>dd</sub>** = 0: ความเร่งเริ่มต้น
- **q<sub>df</sub>** = 0: ความเร็วสุดท้าย
- **q<sub>ddf</sub>** = 0: ความเร่งสุดท้าย
- **T** = 0.5: ระยะเวลารวมของการเคลื่อนที่ (ครึ่งวินาที)

---

## รูปแบบสมการ Quintic Polynomial
สมการ quintic polynomial มีดังนี้:

<p>
q(t) = a<sub>0</sub> + a<sub>1</sub> t + a<sub>2</sub> t<sup>2</sup> + a<sub>3</sub> t<sup>3</sup> + a<sub>4</sub> t<sup>4</sup> + a<sub>5</sub> t<sup>5</sup>
</p>



โดยที่:
- **q(t)** : ค่าตำแหน่ง ที่เวลา t
- **a<sub>0</sub>,a<sub>1</sub>,a<sub>2</sub>,a<sub>3</sub>,a<sub>4</sub>,a<sub>5</sub>** : ค่าสัมประสิทธิ์ที่ต้องการหา

---

ตำแหน่ง **q** คำนวณจากสมการพหุนาม:

<p>
q(t) = a<sub>0</sub> + a<sub>1</sub> t + a<sub>2</sub> t<sup>2</sup> + a<sub>3</sub> t<sup>3</sup> + a<sub>4</sub> t<sup>4</sup> + a<sub>5</sub> t<sup>5</sup>
</p>


ความเร็ว **q<sub>d</sub>** คำนวณจากอนุพันธ์ลำดับแรกของสมการตำแหน่ง:

<p>
q<sub>d</sub>(t) = a<sub>1</sub> + 2a<sub>2</sub> t + 3a<sub>3</sub> t<sup>2</sup> + 4a<sub>4</sub> t<sup>3</sup> + 5a<sub>5</sub> t<sup>4</sup> 
</p>

ความเร่ง**q<sub>dd</sub>** คำนวณจากอนุพันธ์ลำดับที่สองของสมการตำแหน่ง

<p>
q<sub>d</sub>(t) = 2a<sub>2</sub> + 6a<sub>3</sub>t+ 12a<sub>4</sub> t<sup>2</sup> + 20a<sub>5</sub> t<sup>2</sup> 
</p>

---
# Control

ทางผู้จัดทำได้แบ่งบล็อก control ออกเป็น 2 ส่วน ได้แก่:

1. **Revolute joints control**  
   สำหรับควบคุม Rotate base (Joint 1) และ Boom (Joint 2)

2. **Prismatic joints control**  
   สำหรับจำลองเป็นระบบไฮดรอลิกโดยควบคุม Arm (Joint 3) และ Bucket (Joint 4)

---

## การทำงานของฟังก์ชัน

อินพุต (Inputs):

- **q**: ตำแหน่ง ณ เวลาที่กำหนด (t)  
- **q<sub>d</sub>**: ความเร็ว ณ เวลาที่กำหนด (t)  

เอาต์พุต (Outputs):

- **Torque**: แรงบิดที่กระทำต่อ Rotate base (Joint 1) และ Boom (Joint 2)  
- **Force**: แรงที่กระทำต่อ Arm (Joint 3) และ Bucket (Joint 4)  

---

Revolute joints control:

- ประกอบด้วยตัวควบคุม **PID สำหรับตำแหน่ง (Position PID)** และ **PID สำหรับความเร็ว (Velocity PID)**  
- รับสัญญาณความคลาดเคลื่อน (error) จากข้อมูลจริงที่ได้จากแบบจำลอง พร้อมทั้งนำค่าแรงที่ได้จากการคำนวณไดนามิกมาประกอบการควบคุม  
- ให้ผลลัพธ์สุดท้ายอยู่ในรูปของแรงบิด (**Torque**)

![Revolute joints control](Picture/Revolute.png)

---

Prismatic joints control:

- องค์ประกอบของระบบเหมือนกับบล็อก Revolute joints control แต่มีการเพิ่มบล็อกสำหรับแปลงสัญญาณตำแหน่งจากข้อต่อแบบหมุน (**revolute joint**) ให้เป็นตำแหน่งการเคลื่อนที่เชิงเส้น (**prismatic joint**)  
- ใช้หลักการคำนวณระยะการเคลื่อนที่ตาม **กฎของโคไซน์** เพื่อใช้ในการจำลองระบบไฮดรอลิกได้อย่างแม่นยำ  
- บล็อกดังกล่าวจะถูกวางไว้ลำดับก่อนบล็อกควบคุม **PID สำหรับตำแหน่ง (Position PID)**  
- ให้ผลลัพธ์สุดท้ายอยู่ในรูปของแรง (**Force**)
  
---
รูปแบบสมการกฎของโคไซน์:
- สำหรับ Prismatic joints 3: **b<sup>2</sup>** = **a<sup>2</sup>**  + **c<sup>2</sup>**  - **2accos(B)**
- สำหรับ Prismatic joints 4: **a<sup>2</sup>** = **b<sup>2</sup>**  + **c<sup>2</sup>**  - **2bccos(A)**

โดย:
- **b**: ความยาวด้านตรงข้ามมุม B  
- **a** และ **c**: ความยาวของด้านที่อยู่ติดกับมุม B  
- **B**: มุมระหว่างด้าน a และ c

![Cosin](Picture/Cosin.png)
![Prismatic joints control](Picture/Prismatic.png)

---
# DOF model 
## บทนำ
[![Watch the video](Video/simulation.mp4.mp4)](Video/simulation.mp4.mp4)




---
</body>
</html>
