using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using static AIM.WheelController;
using static UnityEngine.Rendering.VolumeComponent;

namespace AIM
{
    /// <summary>
    /// タイヤの操作を行うクラス(先輩からの引継ぎ)
    /// 作成日：2022/不明/不明
    /// 作成者：不明
    /// 更新：2023/05/20～2023/09/22
    /// タイヤが車体に対して生み出す力の計算をすべて変更
    /// 場所：河合奏で検索後"----"に挟まれている箇所
    /// 更新者：河合奏
    /// </summary>
    [Serializable]
    public partial class WheelController : MonoBehaviour
    {
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //先輩のみの個所
        //車体の情報を持つ親オブジェクトのクラス
        private VehicleController vc;

        //ホイールに関係する情報を保持するクラス
        [SerializeField]
        public Wheel wheel;

        //サスペンションのスプリングに関する情報を保持するクラス
        [SerializeField, /*HideInInspector*/]
        public Spring spring;

        //サスペンションのダンパーに関する情報を保持するクラス
        [SerializeField/*, HideInInspector*/]
        public Damper damper;

        [SerializeField]
        public Suspension suspension;   // サスペンションの追加処理用クラスの宣言(2023/08/31_船渡)

        //タイヤの縦方向の物理現象に関する情報を保持するクラス（先輩引継ぎ）
        [SerializeField, HideInInspector]
        public Friction fFriction;

        //タイヤの横方向の物理現象に関する情報を保持するクラス（先輩引継ぎ）
        //[SerializeField, HideInInspector]
        public Friction sFriction;


        //タイヤが触れている地面の情報（先輩引継ぎ）
        [SerializeField]
        private WheelHit[] wheelHits;

        //タイヤが触れることのできるレイヤーマスク（先輩引継ぎ）
        [SerializeField]
        private LayerMask scanIgnoreLayers = Physics.IgnoreRaycastLayer;

        [SerializeField]
        private int forwardScanResolution = 8;

        [SerializeField]
        private int sideToSideScanResolution = 3;

        [SerializeField]
        private bool hasHit = true;
        [SerializeField]
        private bool prevHasHit = true;

        public bool debug;

        [SerializeField]
        public GameObject parent;
        private Rigidbody parentRigidbody;

        public bool useRimCollider = true;

        public enum LRSide
        {
            Left = -1,
            Right = 1,
            Center = 0,
            Auto = 2
        }

        public enum FRSide
        {
            Front = 1,
            Rear = -1,
            Center = 0,
        }

        [SerializeField]
        private LRSide vehicleLRSide = LRSide.Auto;

        [SerializeField]
        private FRSide vehicleFRSide = FRSide.Center;

        public FrictionPreset.FrictionPresetEnum activeFrictionPresetEnum;

        public FrictionPreset activeFrictionPreset;

        public WheelHit wheelHit = new WheelHit();

        public bool singleRay = false;
        public WheelHit singleWheelHit = new WheelHit();

        [HideInInspector]
        public bool trackedVehicle = false;

        [HideInInspector]
        public float trackedOffset = 0;

        private Quaternion steerQuaternion;
        private Quaternion camberQuaternion;
        private Quaternion totalRotation;

        private float boundsX, boundsY, boundsZ, boundsW;
        private float stepX, stepY;
        private float rayLength;
        private int minDistRayIndex;

        private WheelHit wheelRay;
        private float n;
        private float minWeight = Mathf.Infinity;
        private float maxWeight = 0f;
        private float weightSum = 0f;
        private int validCount = 0;

        [NonSerialized]
        private Vector3 hitPointSum = Vector3.zero;
        [NonSerialized]
        private Vector3 normalSum = Vector3.zero;
        [NonSerialized]
        private Vector3 point = new Vector3();
        [NonSerialized]
        private Vector3 normal = new Vector3();
        private float weight = 0;

        private float forwardSum = 0;
        private float sideSum = 0;
        private float angleSum = 0;
        private float offsetSum = 0;

        private Vector3 transformUp;
        private Vector3 transformForward;
        private Vector3 transformRight;
        private Vector3 transformPosition;
        private Quaternion transformRotation;

        public bool applyForceToOthers = false;
        public float maxPutDownForce;

        private Vector3 origin;
        private Vector3 alternateForwardNormal;
        public Vector3 totalForce;
        private Vector3 forcePoint;
        private Vector3 hitDir;
        private Vector3 predictedDistance;
        private Vector3 wheelDown;
        private Vector3 offsetPrecalc;
        private float prevForwardSpeed;
        private float prevFreeRollingAngularVelocity;
        private Quaternion axleRotation;

        private Vector3 projectedNormal;
        private Vector3 projectedAltNormal;

        private Transform trans;
        private Transform visualTrans;

        private NativeArray<RaycastHit> raycastHits;
        private NativeArray<RaycastCommand> raycastCommands;
        private JobHandle raycastJobHandle;

        private WheelHit wr;
        private RaycastCommand rc = new RaycastCommand();
        private RaycastHit tmpHit;

        private float inertia;
        private float motorForce;
        private float brakeForce;
        private float angularDeceleration;
        private float decelerationForce;
        private float decelerationDelta;
        private float accelerationForce;
        private float accelerationDelta;

        //河合奏追記-------------------
        private Vector3 traction;                        //トラクション（車輪に対して前に進む力）
        public Vector3 rollingResistance;               //転がり抵抗
        public Vector3 corneringforce;                  //コーナリングフォース
        private Vector3 Brake;                          //ブレーキ
        public float load;                              //荷重
        public AnimationCurve corneringForceCurve;       //コーナリングパワーを取得する用のカーブ

        public Vector3 NomalForward;
        public Vector3 NomalRight;

        public Vector3 Forward;
        public Vector3 Right;

        [SerializeField] public float CurveFactor = 1.35f;         //カーブする強さの補正
        [SerializeField] public float MaxStopFriction = 0.28f;    //スリップのしやすさ補正
        [SerializeField] public float MoveFriction = 0.28f;    //スリップのしやすさ補正

        public int EfectPower = 0;                                    //ハンドルの重さ

        public float RAccel = 0.0f;
        public float preRVelocity = 0.0f;

        public float preSlipAngle = 0.0f;
        public float SlipVelocity = 0.0f;

        private bool isAppend = false; //デバッグ用の数値の記録を上書き or 追記
        //-----------------------------

        private Vector3 wheelHitPoint;
        private Vector3 raycastHitNormal;
        public Vector3 contactVelocity;

        #region 定数
        const float RAY_LENGTH_COEFF = 2.1f;
        const float TIRE_RADIUS_COEFF = 0.03f;
        const float SPRING_LENGTH_TIME_COEFF = 8.0f;
        const float SMOOTH_SPEED_TIME_COEFF = 2.0f;
        const float SPRING_OVERFLOW_COEFF = 20.0f;
        const float LOWER_LIMIT_MAX = 3.0f;
        const float F_SLIP_COEFF = 80.0f;
        const float F_FORCE_COEFF_CONST = 1.3f;
        const float F_CLAMPED_FORWARD_SPEED_MIN = 0.22f;
        const float CLAMPED_SLIP_MIN = 0.05f;
        const float MAXPUT_DOWNFORCE_COEFF = 1.3f;
        const float VELOCITY_TO_RPM = 9.55f;
        const float HITPOINT_COEFF = 3.0f;
        const float FORCEPOINT_COEFF = 4.0f;
        const int STEER_ROUND = 3;
        #endregion

        private void Awake()
        {
            trans = transform;

            // タイヤとの当たり判定を切りたいならここで設定
            // ORでOnにして、反転させる
            scanIgnoreLayers = scanIgnoreLayers | (1 << 2);
            scanIgnoreLayers = scanIgnoreLayers | (1 << LayerMask.NameToLayer("ConesGround"));
            scanIgnoreLayers = ~scanIgnoreLayers;

            Initialize();

            if (wheel.visual != null)
            {
                visualTrans = wheel.visual.transform;
                wheel.worldPosition = visualTrans.position;
                wheel.up = visualTrans.up;
                wheel.forward = visualTrans.forward;
                wheel.right = visualTrans.right;
            }

            if (wheel.nonRotating != null)
            {
                wheel.nonRotatingPostionOffset = trans.InverseTransformDirection(wheel.nonRotating.transform.position - visualTrans.position);
            }

            wheel.Initialize(this);

            InitializeScanParams();

            parentRigidbody = parent.GetComponent<Rigidbody>();

            spring.length = spring.maxLength * 0.5f;

            vc = GetComponentInParent<VehicleController>();
        }
        public void InitializeScanParams()
        {
            boundsX = -wheel.width * 0.5f;
            boundsY = -wheel.tireRadius;

            boundsZ = wheel.width * 0.5f + 0.000001f;
            boundsW = wheel.tireRadius + 0.000001f;

            stepX = sideToSideScanResolution == 1 ? 1 : (wheel.width) / (sideToSideScanResolution - 1);
            stepY = forwardScanResolution == 1 ? 1 : (wheel.tireRadius * 2.0f) / (forwardScanResolution - 1);

            // レイの初期化
            int n = forwardScanResolution * sideToSideScanResolution;
            wheelHits = new WheelHit[n];

            int w = 0;
            for (float x = boundsX; x <= boundsZ; x += stepX)
            {
                int h = 0;
                for (float y = boundsY; y <= boundsW; y += stepY)
                {
                    int index = w * forwardScanResolution + h;

                    WheelHit wr = new WheelHit();
                    wr.angleForward = Mathf.Asin(y / (wheel.tireRadius + 0.000001f));
                    wr.curvatureOffset = Mathf.Cos(wr.angleForward) * wheel.tireRadius;

                    float xOffset = x;
                    if (sideToSideScanResolution == 1) xOffset = 0;
                    wr.offset = new Vector2(xOffset, y);
                    wheelHits[index] = wr;

                    h++;
                }
                w++;
            }

            if (raycastCommands.Length > 0) raycastCommands.Dispose();
            if (raycastHits.Length > 0) raycastHits.Dispose();

            raycastCommands = new NativeArray<RaycastCommand>(n, Allocator.Persistent);
            raycastHits = new NativeArray<RaycastHit>(n, Allocator.Persistent);
        }


        public void FixedUpdate()
        {
            prevHasHit = hasHit;

            transformPosition = trans.position;
            transformRotation = trans.rotation;
            transformForward = trans.forward;
            transformRight = trans.right;
            transformUp = trans.up;

            if (!parentRigidbody.IsSleeping())
            {
                // hasHitの更新
                HitUpdate();
                SuspensionUpdate();
                CalculateWheelDirectionsAndRotations();
                WheelUpdate();
                FrictionUpdate();
                UpdateForces();
            }

            //=============================
            //東樹追加
            SetActiveFrictionPreset(activeFrictionPresetEnum);
            //=============================
        }


        private void OnDisable()
        {
            OnDestroy();
        }

        private void OnDestroy()
        {
            try
            {
                raycastCommands.Dispose();
                raycastHits.Dispose();
            }
            catch { }
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //ハンドルからの情報を受け取りタイヤの回転に反映
        private void CalculateWheelDirectionsAndRotations()
        {
            //ハンドルからステアリング角度を受け取る
            steerQuaternion = Quaternion.AngleAxis(wheel.steerAngle, transformUp);
            camberQuaternion = Quaternion.AngleAxis(-(int)vehicleLRSide * wheel.camberAngle, transformForward);
            totalRotation = steerQuaternion * camberQuaternion;

            //河合変更--------------------------------------------------
            //デフォルトのタイヤの角度にステアリングの角度を反映
            //リアホイールには反映しない
            if ((int)vehicleFRSide == 1)
            {
                wheel.up = steerQuaternion * transformUp;
                wheel.forward = steerQuaternion * transformForward;
                wheel.right = steerQuaternion * transformRight;
            }
            else
            {
                wheel.up = transformUp;
                wheel.forward = transformForward;
                wheel.right = transformRight;
            }
            //--------------------------------------------------------------

            // vehicleLRSideはタイヤの位置
            // Leftは-1 Rightは1
            wheel.inside = wheel.right * -(int)vehicleLRSide;
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //先輩のみの個所
        //地面のヒット判定
        private void HitUpdate()
        {
            float minDistance = Mathf.Infinity;
            wheelDown = -wheel.up;

            float distanceThreshold = spring.maxLength - spring.length;
            rayLength = wheel.tireRadius * RAY_LENGTH_COEFF + distanceThreshold;

            // スプリングの長さ分下に、タイヤの半径分上に、あとはタイヤの向きによって変わる（insideは右と左、rimOffsetは0.008）
            // 大体 Y方向に0.2プラス
            offsetPrecalc = transformPosition - transformUp * spring.length + wheel.up * wheel.tireRadius - wheel.inside * wheel.rimOffset;

            int validHitCount = 0;
            minDistRayIndex = -1;
            hasHit = false;

            if (singleRay)
            {
                singleWheelHit.valid = false;

                // 下（地面方向）へ向けて、Rayを発射
                // scanIgnoreLayersはIgnoreRaycast以外に反応するように設定
                bool grounded = Physics.Raycast(offsetPrecalc, wheelDown, out singleWheelHit.raycastHit, rayLength + wheel.tireRadius, scanIgnoreLayers);

                // 地面に付いているなら
                if (grounded)
                {
                    // ほぼ0に近い値になる
                    float distanceFromTire = singleWheelHit.raycastHit.distance - wheel.tireRadius - wheel.tireRadius;

                    if (distanceFromTire > distanceThreshold) return;
                    singleWheelHit.valid = true;

                    hasHit = true;

                    // 結果を保存
                    singleWheelHit.distanceFromTire = distanceFromTire;

                    wheelHit.raycastHit = singleWheelHit.raycastHit;
                    wheelHit.angleForward = singleWheelHit.angleForward;
                    wheelHit.distanceFromTire = singleWheelHit.distanceFromTire;
                    wheelHit.offset = singleWheelHit.offset;
                    wheelHit.weight = singleWheelHit.weight;
                    wheelHit.curvatureOffset = singleWheelHit.curvatureOffset;

                    wheelHit.groundPoint = wheelHit.raycastHit.point;
                    wheelHit.raycastHit.point += wheel.up * wheel.tireRadius;
                    wheelHit.curvatureOffset = wheel.tireRadius;

                    // サスペンション処理のために地面の座標を保存する(0831_船渡)
                    suspension.GroundPos = wheelHit.raycastHit.point.y;
                }
            }
            else
            {
                int n = wheelHits.Length;

                if (!raycastCommands.IsCreated) raycastCommands = new NativeArray<RaycastCommand>(n, Allocator.Persistent);
                if (!raycastHits.IsCreated) raycastHits = new NativeArray<RaycastHit>(n, Allocator.Persistent);

                for (int i = 0; i < n; i++)
                {
                    // 結果の保存
                    wr = wheelHits[i];
                    wr.valid = false;

                    origin.x = wheel.forward.x * wr.offset.y + wheel.right.x * wr.offset.x + offsetPrecalc.x;
                    origin.y = wheel.forward.y * wr.offset.y + wheel.right.y * wr.offset.x + offsetPrecalc.y;
                    origin.z = wheel.forward.z * wr.offset.y + wheel.right.z * wr.offset.x + offsetPrecalc.z;

                    rc.from = origin;
                    rc.direction = wheelDown;
                    rc.distance = rayLength + wr.curvatureOffset;
                    rc.layerMask = scanIgnoreLayers;
                    rc.maxHits = 1;
                    raycastCommands[i] = rc;
                }
                raycastJobHandle = RaycastCommand.ScheduleBatch(raycastCommands, raycastHits, 8);
                raycastJobHandle.Complete();

                for (int i = 0; i < n; i++)
                {
                    tmpHit = raycastHits[i];

                    wr = wheelHits[i];
                    wr.valid = false;

                    if (tmpHit.distance > 0.0f)
                    {
                        float distanceFromTire = tmpHit.distance - wr.curvatureOffset - wheel.tireRadius;

                        if (distanceFromTire > distanceThreshold) continue;

                        wr.valid = true;
                        wr.raycastHit = tmpHit;
                        wr.distanceFromTire = distanceFromTire;

                        validHitCount++;

                        if (distanceFromTire < minDistance)
                        {
                            minDistance = distanceFromTire;
                            minDistRayIndex = i;
                        }
                    }

                    wheelHits[i] = wr;
                }
                CalculateAverageWheelHit();
            }

            // 摩擦力方向、タイヤの正面方向と横方向のベクトル
            if (hasHit || true)
            {
                wheelHit.forwardDir = Vector3.Normalize(Vector3.Cross(wheelHit.normal, -wheel.right));
                wheelHit.sidewaysDir = Quaternion.AngleAxis(90f, wheelHit.normal) * wheelHit.forwardDir;
            }
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //先輩のみの個所
        //地面のヒット後の情報整理？
        private void CalculateAverageWheelHit()
        {
            int count = 0;

            n = wheelHits.Length;

            minWeight = Mathf.Infinity;
            maxWeight = 0f;
            weightSum = 0f;
            validCount = 0;

            hitPointSum = Vector3.zero;
            normalSum = Vector3.zero;
            weight = 0;

            forwardSum = 0;
            sideSum = 0;
            angleSum = 0;
            offsetSum = 0;
            validCount = 0;

            for (int i = 0; i < n; i++)
            {
                wheelRay = wheelHits[i];
                if (wheelRay.valid)
                {
                    weight = wheel.tireRadius - wheelRay.distanceFromTire;
                    weight = weight * weight * weight * weight * weight;

                    if (weight < minWeight) minWeight = weight;
                    else if (weight > maxWeight) maxWeight = weight;

                    weightSum += weight;
                    validCount++;

                    normal = wheelRay.raycastHit.normal;
                    point = wheelRay.raycastHit.point;

                    hitPointSum.x += point.x * weight;
                    hitPointSum.y += point.y * weight;
                    hitPointSum.z += point.z * weight;

                    normalSum.x += normal.x * weight;
                    normalSum.y += normal.y * weight;
                    normalSum.z += normal.z * weight;

                    forwardSum += wheelRay.offset.y * weight;
                    sideSum += wheelRay.offset.x * weight;
                    angleSum += wheelRay.angleForward * weight;
                    offsetSum += wheelRay.curvatureOffset * weight;

                    count++;
                }
            }
            if (validCount == 0 || minDistRayIndex < 0)
            {
                hasHit = false;
                return;
            }
            wheelHit.raycastHit = wheelHits[minDistRayIndex].raycastHit;
            wheelHit.raycastHit.point = hitPointSum / weightSum;
            wheelHit.offset.y = forwardSum / weightSum;
            wheelHit.offset.x = sideSum / weightSum;
            wheelHit.angleForward = angleSum / weightSum;
            wheelHit.raycastHit.normal = Vector3.Normalize(normalSum / weightSum);
            wheelHit.curvatureOffset = offsetSum / weightSum;
            wheelHit.raycastHit.point += wheel.up * wheelHit.curvatureOffset;
            wheelHit.groundPoint = wheelHit.raycastHit.point - wheel.up * wheelHit.curvatureOffset;

            hasHit = true;
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //チームメンバーの個所
        private void SuspensionUpdate()
        {
            spring.prevOverflow = spring.overflow;  // 前回の値の保存(0817)
            spring.overflow = 0f;                   // 値の初期化(0817)

            //地面に接していて、地面の法線が少しでもup方向をむいているとき
            if ((hasHit || true) && Vector3.Dot(wheelHit.raycastHit.normal, transformUp) > 0.1f)
            {

                spring.bottomedOut = spring.overExtended = false;

                if (singleRay)
                {
                    spring.targetPoint = wheelHit.raycastHit.point - wheel.right * wheel.rimOffset * (int)vehicleLRSide;
                }
                else
                {
                    spring.targetPoint = wheelHit.raycastHit.point
                         + wheel.up * wheel.tireRadius * TIRE_RADIUS_COEFF
                        - wheel.forward * wheelHit.offset.y
                        - wheel.right * wheelHit.offset.x
                        - wheel.right * wheel.rimOffset * (int)vehicleLRSide;
                }

                //車体の傾きに影響している？
                spring.length = -trans.InverseTransformPoint(spring.targetPoint).y;
                // ------------------------------荷重適用(2023/08/24_船渡_0915現在未使用)------------------------------
                // 荷重の取得
                float load = LoadForceDistribution;
                Debug.Log($"~_load : {load}");

                // 取得した荷重が設定された最大荷重を超えていたら
                if (load > spring.MaxLoad)
                {
                    // 最大荷重に補正する
                    load = spring.MaxLoad;
                }
                // 取得した荷重が設定された最小荷重を超えていたら
                else if (load < spring.MinLoad)
                {
                    // 最小荷重に補正する
                    load = spring.MinLoad;
                }
                // 0から指定された値にスケーリング
                float ScaleLoad = (load - spring.MinLoad) / (spring.MaxLoad - spring.MinLoad) * spring.LoadCorrection;

                // もともとある長さに値を足して0から1の間に値を補正する
                //spring.length = Mathf.Clamp01(spring.length + ScaleLoad);

                // ------------------------------------------------------------------------------

                //if(!isFront)
                //{
                //	spring.length = 0.05f;
                //}

                // スプリングの長さが0.0f未満の場合(0817)
                if (spring.length < 0.0f)
                {
                    // 突破分を保存(0817)
                    spring.overflow = -spring.length;
                    // 長さを0にする(0817)
                    spring.length = 0f;
                    // 沈み切ったフラグ(?)をオンにする(0817)
                    spring.bottomedOut = true;
                }

                // スプリングの長さが最大の長さを超えたら(0817)
                else if (spring.length > spring.maxLength)
                {
                    // 地面に当たっているフラグをオフにする(0817)
                    hasHit = false;
                    // 長さを最大の長さにする(0817)
                    spring.length = spring.maxLength;
                    // スプリングが伸び切ったフラグをオフにする(0817)
                    spring.overExtended = true;
                }
            }
            else
            {
                spring.length = Mathf.Lerp(spring.length, spring.maxLength, Time.fixedDeltaTime * SPRING_LENGTH_TIME_COEFF);
            }
            // DamperForceの計算に使われる
            spring.velocity = (spring.length - spring.prevLength) / Time.fixedDeltaTime;

            // スプリングの伸び具合
            spring.compressionPercent = (spring.maxLength - spring.length) / spring.maxLength;

            // forceCurveは直線ではなく、ほんの少しだけ晩熟型のグラフ
            // なので、ほぼ  maxForce*spring.compressionPercent
            spring.force = spring.maxForce * spring.forceCurve.Evaluate(spring.compressionPercent);

            // 超過した速度(量?)を0にする(0817)
            spring.overflowVelocity = 0f;

            // スプリングがオーバーフローしていた場合(0817)
            if (spring.overflow > 0)
            {
                // 超過速度を設定する(超過分 - 前回の超過分 / デルタタイム)(0817)
                spring.overflowVelocity = (spring.overflow - spring.prevOverflow) / Time.fixedDeltaTime;
                // 下側への力(?)を設定する(質量 * -y軸の重力 * スプリングの速度 + スプリングの超過した速度 + スプリングの超過量 * オーバーフロー時の係数) * 0.5f(0817)
                spring.bottomOutForce = parentRigidbody.mass * -Physics.gravity.y * ((spring.velocity + spring.overflowVelocity) + spring.overflow * SPRING_OVERFLOW_COEFF) * 0.5f;

                // 計算した力を加える(0817)
                parentRigidbody.AddForceAtPosition(spring.bottomOutForce * Vector3.up, transformPosition);
                //Debug.Log("2_Call");
            }
            else
            {
                // ダンパーにかかる最大の力の計算(スプリングの長さが前回以下の場合unitBumpForceをそうでない場合unitReboundForceを代入)(0817)
                damper.maxForce = spring.length <= spring.prevLength ? damper.unitBumpForce : damper.unitReboundForce;

                // スプリングの長さが前回以下の場合
                if (spring.length <= spring.prevLength)
                    //ダンパーの力 = ダンパーにかかる力に吸収する力 * 取得したアニメーションカーブ(スプリングの速度(力 ?)を渡す)の値(0817)
                    damper.force = damper.unitBumpForce * damper.dampingCurve.Evaluate(Mathf.Abs(spring.velocity));


                else
                    //ダンパーの力 = -ダンパーの反発 * 取得したアニメーションカーブ(スプリングの速度(力 ?)を渡す)の値(0817)
                    damper.force = -damper.unitReboundForce * damper.dampingCurve.Evaluate(Mathf.Abs(spring.velocity));

                // ------------------2023/08/30_船渡------------------
                // 力が一定の値を超えたら補正する
                if (spring.force > 7000.0f)
                {
                    //spring.force = 7000.0f;
                }
                // 力が一定の値を下回ったら補正する
                else if (spring.force < 3000.0f)
                {
                    //spring.force = 3000.0f;
                }
                // ----------------------------------------------------
            }
            // 前回の長さに今回に長さを代入する(0817)
            spring.prevLength = spring.length;

            if (hasHit)
            {
                // ----------------------------段差に対応するためのサスペンション処理(2023/08/30_船渡)------------------------
                // サスペンションの位置を計算する
                // ホイールの中心座標 - 地面の座標
                suspension.SuspensionHeight = transformPosition.y - suspension.GroundPos;

                if (suspension.SuspensionHeight < 0.0f)
                {
                    suspension.SuspensionHeight = 0.1f;
                }
                if (suspension.SuspensionHeight > 1.0f)
                {
                    suspension.SuspensionHeight = 1.0f;
                }

                // サスペンションの力が加わる位置を計算する(別のAddForceが必要な場合のみ使用する)
                //suspension.ForcePoint = transformPosition - Vector3.up * suspension.SuspensionHeight;

                float supringLength = suspension.DefaultLength - suspension.SuspensionHeight;

                // スプリング(ばね)の力の計算
                // スプリングの変位 * ばね定数
                float springForce = supringLength * suspension.SpringConstant;

                // サスペンションの速度を取得
                float suspensionVelocity = parentRigidbody.velocity.y;

                // サスペンションの状態によって使用する減衰係数を変更する
                float damperConstant = suspensionVelocity > 0 ? suspension.ExtensionDamperConstant : suspension.CompressionDamperConstant;

                // ダンパーの減衰力の計算
                // ホイールの相対的な速度のy軸 * ダンパーの減衰係数
                float damperForce = -suspension.SuspensionHeight * damperConstant * suspensionVelocity;

                //Debug.Log($"<_spring.SuspensionHeight : {spring.SuspensionHeight}");
                //Debug.Log($"<_WheelPos.y - spring.ForcePos.y : {WheelPos.y - spring.ForcePos.y}");
                //Debug.Log($"<_spring.ForcePos : {suspension.ForcePoint}");
                //Debug.Log($"<_displacement : {displacement}");
                //Debug.Log($"<_springForce : {springForce}");
                //Debug.Log($"<_damperForce : {damperForce}");

                // サスペンションの最終的な力の計算
                suspension.SuspensionForce = springForce + damperForce + LoadForceDistribution * suspension.LoadCorrection;

                //Debug.Log($"<_spring.suspensionForce : {suspension.SuspensionForce}");

                //Debug.Log($"+_Ground : {suspension.GroundPos}");
                //Debug.Log($"+_WheelPoint : {transformPosition.y}");
                // ----------------------------------------------------------------------------------------------------------
            }
            else
            {
                suspension.SuspensionForce = 0.0f;
            }
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //先輩のみの個所
        //タイヤの回転速度反映
        private void WheelUpdate()
        {
            wheel.prevWorldPosition = wheel.worldPosition;
            wheel.worldPosition = transformPosition - transformUp * spring.length - wheel.inside * wheel.rimOffset;

            wheel.prevVelocity = wheel.velocity;
            wheel.velocity = parentRigidbody.GetPointVelocity(wheel.worldPosition);
            wheel.acceleration = (wheel.velocity - wheel.prevVelocity) / Time.fixedDeltaTime;

            wheel.camberAngle = wheel.camberCurve.Evaluate(spring.length / spring.maxLength);

            wheel.tireLoad = Mathf.Clamp(spring.force + damper.force, 0.0f, Mathf.Infinity);
            if (hasHit || true) wheelHit.force = wheel.tireLoad;

            wheel.rotationAngle = (wheel.rotationAngle % 360.0f) + (wheel.angularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime);

            axleRotation = Quaternion.AngleAxis(wheel.rotationAngle, transformRight);

            wheel.worldRotation = totalRotation * axleRotation * transformRotation;

            // タイヤ見た目オブジェクトを移動・回転
            if (wheel.visual != null)
            {
                // アクセルの影響を見た目に反映しない場合
                if (wheel.isVisualNonSteer)
                {
                    visualTrans.localRotation = Quaternion.Euler(-wheel.worldRotation.eulerAngles.x, 0.0f, 0.0f);
                }
                else
                {
                    visualTrans.rotation = wheel.worldRotation;
                }

                if (trackedVehicle)
                {
                    visualTrans.position = wheel.worldPosition - transformUp * trackedOffset;
                }
                else
                {
                    visualTrans.position = wheel.worldPosition;
                }

            }

            // 回転させないオブジェクトの処理（ブレーキなど）
            if (wheel.nonRotating != null)
            {
                wheel.nonRotating.transform.rotation = totalRotation * transformRotation;

                if (trackedVehicle)
                {
                    wheel.nonRotating.transform.position = wheel.worldPosition + trans.TransformDirection(totalRotation * wheel.nonRotatingPostionOffset) - transformUp * trackedOffset;
                }
                else
                {
                    wheel.nonRotating.transform.position = wheel.worldPosition + trans.TransformDirection(totalRotation * wheel.nonRotatingPostionOffset);
                }
            }

            if (useRimCollider)
            {
                wheel.rim.transform.position = wheel.worldPosition;
                wheel.rim.transform.rotation = steerQuaternion * camberQuaternion * transformRotation;
            }
        }

        //タイヤの見た目上の回転速度の反映
        private void VisualOnlyUpdate()
        {
            spring.targetPoint = wheelHit.raycastHit.point - wheel.right * wheel.rimOffset * (int)vehicleLRSide;
            spring.length = -trans.InverseTransformPoint(spring.targetPoint).y;
            spring.length = Mathf.Clamp(spring.length, 0, spring.maxLength);
            wheel.camberAngle = wheel.camberCurve.Evaluate(spring.length / spring.maxLength);

            wheel.worldPosition = transformPosition - transformUp * spring.length - wheel.inside * wheel.rimOffset;
            wheel.worldRotation = totalRotation * transformRotation;

            wheel.velocity = (wheel.worldPosition - wheel.prevWorldPosition) / Time.fixedDeltaTime;
            wheel.angularVelocity = transform.InverseTransformVector(wheel.velocity).z / wheel.tireRadius;
            wheel.rotationAngle = (wheel.rotationAngle % 360.0f) + (wheel.angularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime);
            steerQuaternion = Quaternion.AngleAxis(wheel.steerAngle, transformUp);
            axleRotation = Quaternion.AngleAxis(wheel.rotationAngle, transformRight);
            wheel.worldRotation = steerQuaternion * axleRotation * transformRotation;

            wheel.prevWorldPosition = wheel.worldPosition;

            if (wheel.visual != null)
            {
                visualTrans.rotation = wheel.worldRotation;

                if (trackedVehicle)
                {
                    visualTrans.position = wheel.worldPosition - transformUp * trackedOffset;
                }
                else
                {
                    visualTrans.position = wheel.worldPosition;
                }

            }
            if (wheel.nonRotating != null)
            {
                wheel.nonRotating.transform.rotation = totalRotation * transformRotation;

                if (trackedVehicle)
                {
                    wheel.nonRotating.transform.position = wheel.worldPosition + trans.TransformDirection(totalRotation * wheel.nonRotatingPostionOffset) - transformUp * trackedOffset;
                }
                else
                {
                    wheel.nonRotating.transform.position = wheel.worldPosition + trans.TransformDirection(totalRotation * wheel.nonRotatingPostionOffset);
                }
            }
        }
        //----------------------------------------------------------------------------------------------------------------------------------------------------------

        private void FrictionUpdate()
        {
            //前回の摩擦速度を保存（河合奏更新/2023/07/14）
            prevForwardSpeed = fFriction.speed;

            //タイヤに対しての各種ノーマルベクトルをY軸の情報だけ抜いて保存（河合奏更新/2023/07/14）
            NomalForward = wheel.forward;
            NomalForward.y = 0.0f;
            NomalRight = wheel.right;

            // 引数のワールド座標における、速度を返す
            // 返ってくる速度は、RigidBodyのVelocityと、Anguler
            // wheelHit.raycastHit.pointは、タイヤから真下に向けた時の接地位置
            contactVelocity = parentRigidbody.GetPointVelocity(wheelHit.raycastHit.point);

            //タイヤの現在の移動スピードを縦横で分解（河合奏更新/2023/07/14）
            fFriction.speed = Vector3.Dot(NomalForward, contactVelocity);
            sFriction.speed = Vector3.Dot(NomalRight, contactVelocity);

            //----------------------------------------------------------------------------------------------------------------------------------------------------------
            //スリップ角の計算（河合奏更新/2023/08/28）
            SlipVelocity = sFriction.slip - preSlipAngle;
            preSlipAngle = sFriction.slip;
            sFriction.slip = 0.0f;
            sFriction.force = 0.0f;

            //タイヤがどの方向に進んでいるのかを求める
            if (fFriction.speed > 0.0f)
            {
                sFriction.slip = (Mathf.Atan(-sFriction.speed / fFriction.speed) * Mathf.Rad2Deg);
            }
            else
            {
                sFriction.slip = (Mathf.Atan(sFriction.speed / -fFriction.speed) * Mathf.Rad2Deg);
                if (Mathf.Sign(SlipVelocity) == Mathf.Sign(sFriction.slip))
                {
                    sFriction.slip += Mathf.Sign(sFriction.slip) * 180.0f;
                }
                else
                {
                    sFriction.slip += -Mathf.Sign(sFriction.slip) * 180.0f;
                }
            }
            //----------------------------------------------------------------------------------------------------------------------------------------------------------

            //----------------------------------------------------------------------------------------------------------------------------------------------------------
            //タイヤの回転速度計算（先輩のみ記入）
            wheel.freeRollingAngularVelocity = fFriction.speed / wheel.tireRadius;

            inertia = wheel.mass * wheel.tireRadius * wheel.tireRadius;
            motorForce = wheel.motorTorque / wheel.tireRadius;
            brakeForce = Mathf.Abs(wheel.brakeTorque / wheel.tireRadius);

            float clampedSlip = Mathf.Clamp(Mathf.Abs(fFriction.slip), CLAMPED_SLIP_MIN, Mathf.Infinity);

            if (!trackedVehicle)
            {
                maxPutDownForce = activeFrictionPreset.Curve.Evaluate(clampedSlip) * wheel.tireLoad * fFriction.forceCoefficient * MAXPUT_DOWNFORCE_COEFF;
            }
            else
            {
                maxPutDownForce = wheel.tireLoad * fFriction.forceCoefficient * MAXPUT_DOWNFORCE_COEFF;
            }
            // 減速力の設定
            decelerationForce = Mathf.Sign(motorForce) * Mathf.Clamp(maxPutDownForce - Mathf.Abs(motorForce), 0.0f, Mathf.Infinity);
            decelerationForce += wheel.dragForce;
            decelerationForce += brakeForce;
            decelerationDelta = inertia == 0.0f ? 0.0f : ((decelerationForce * wheel.tireRadius) / inertia) * Time.fixedDeltaTime;

            // 加速力の設定
            accelerationForce = Mathf.Sign(motorForce) * Mathf.Clamp((Mathf.Abs(motorForce) - maxPutDownForce), 0.0f, Mathf.Infinity);
            accelerationDelta = inertia == 0.0f ? 0.0f : ((accelerationForce * wheel.tireRadius) / inertia) * Time.fixedDeltaTime;

            // 角速度の計算
            wheel.residualAngularVelocity += accelerationDelta - decelerationDelta;

            if (motorForce >= 0.0f)
                wheel.residualAngularVelocity = Mathf.Clamp(wheel.residualAngularVelocity, 0.0f, Mathf.Infinity);
            else
                wheel.residualAngularVelocity = Mathf.Clamp(wheel.residualAngularVelocity, -Mathf.Infinity, 0.0f);

            wheel.residualAngularVelocity = prevFreeRollingAngularVelocity;

            // タイヤの角速度（回転速度）を設定
            wheel.angularVelocity = wheel.freeRollingAngularVelocity + wheel.residualAngularVelocity;

            angularDeceleration = inertia == 0 ? 0 : -Mathf.Sign(wheel.angularVelocity) * ((brakeForce * wheel.tireRadius) / inertia) * Time.fixedDeltaTime;

            // ブレーキ時の、タイヤの角速度（回転速度）を減らす
            if (wheel.angularVelocity < 0.0f)
                wheel.angularVelocity = Mathf.Clamp(wheel.angularVelocity + angularDeceleration, -Mathf.Infinity, 0.0f);
            else
                wheel.angularVelocity = Mathf.Clamp(wheel.angularVelocity + angularDeceleration, 0.0f, Mathf.Infinity);

            wheel.residualAngularVelocity = Mathf.Sign(wheel.residualAngularVelocity) * Mathf.Clamp(Mathf.Abs(wheel.residualAngularVelocity), 0.0f, 200.0f);

            if (brakeForce != 0.0f && Mathf.Abs(motorForce) < brakeForce && brakeForce < maxPutDownForce)
                wheel.angularVelocity = wheel.freeRollingAngularVelocity;

            if (trackedVehicle)
                wheel.angularVelocity = wheel.freeRollingAngularVelocity;

            // 角速度をRPMに変換
            // タイヤのRPMの完成
            // マジックナンバー0,53f（北村追記）
            wheel.rpm = wheel.angularVelocity * VELOCITY_TO_RPM * 0.53f;

            if (hasHit || true)
            {
                wheelHit.forwardSlip = fFriction.slip;
                wheelHit.sidewaysSlip = sFriction.slip;
            }
            prevFreeRollingAngularVelocity = wheel.freeRollingAngularVelocity;
            //---------------------------------------------------------------------------------------------------------------------------------------------------------
        }

        private void UpdateForces()
        {
            // 地面にいるときの処理
            if (hasHit)
            {
                wheelHitPoint = wheelHit.point;
                raycastHitNormal = wheelHit.raycastHit.normal;

                if (Vector3.Dot(raycastHitNormal, transformUp) > 0.1f)
                {
                    // サスペンションの力の大きさ（0～Infinityの範囲で、spring.force + damper.force）
                    // サスペンション追加計算処理を追記(0831_船渡）
                    float suspensionForceMagnitude = Mathf.Clamp(spring.force + damper.force - suspension.SuspensionForce, 0.0f, Mathf.Infinity); ;

                    //タイヤが地面から車両を押す力はここで計算
                    totalForce = suspensionForceMagnitude * Vector3.up;

                    //タイヤが生み出す力が車体を押す力の統合
                    totalForce += WheelToBodyAddForce + CorneringForce + RollingResistance - BrakeForce;
 
                    //車輪がついているRigitBodyの座標計算
                    forcePoint.x = (wheelHitPoint.x * HITPOINT_COEFF + spring.targetPoint.x) / FORCEPOINT_COEFF;
                    forcePoint.y = (wheelHitPoint.y * HITPOINT_COEFF + spring.targetPoint.y) / FORCEPOINT_COEFF;
                    forcePoint.z = (wheelHitPoint.z * HITPOINT_COEFF + spring.targetPoint.z) / FORCEPOINT_COEFF;

                    //デバック用の記録保存
                    /*Writing(WheelToBodyAddForce, "TortalForceReport.txt");
                    Writing(RollingResistance, "RollingResistanceReport.txt");
                    Writing(CorneringForce, "CorneringForce.txt");*/
                    //===========================================================================
                    // ここで行われるAddForceが実際に車を動かす力
                    // totalForceのいずれかの値がNaNの場合、処理を行わないようにする。
                    if (float.IsNaN(totalForce.x) || float.IsNaN(totalForce.y) || float.IsNaN(totalForce.z))
                    {

                    }
                    else
                    {
                        // 別で計算したサスペンションの値を足す(2023/08/30_船渡)
                        //totalForce -= suspension.SuspensionForce * Vector3.up;

                        // 車両に力を適用する
                        parentRigidbody.AddForceAtPosition(totalForce, forcePoint);
                    }
                    //parentRigidbody.AddForceAtPosition(-suspension.SuspensionForce * Vector3.up, suspension.ForcePoint);

                    spring.carForce_x = totalForce.x;
                    spring.carForce_y = totalForce.y;
                    spring.carForce_z = totalForce.z;

                    spring.BeforeForce = totalForce;
                    //===========================================================================
                }
            }
        }


        #region Classes
        /*****************************/
        /* CLASSES                   */
        /*****************************/

        [System.Serializable]
        public class Friction
        {
            public float forceCoefficient = 1.1f;
            public float slipCoefficient = 1;
            public float maxForce;
            public float slip = 0.0f;
            public float speed = 0.0f;
            public float force;
        }

        [System.Serializable]
        public class Damper
        {
            public AnimationCurve dampingCurve = null;
            public float unitBumpForce = 1000.0f;
            public float unitReboundForce = 1400.0f;
            public float force;
            public float maxForce;
        }

        [System.Serializable]
        public class Spring
        {
            public float maxLength = 0.3f;
            public AnimationCurve forceCurve = null;
            public float maxForce = 220000.0f;

            public float length;
            public float prevLength;
            public float compressionPercent;
            public float force;
            public float velocity;
            public Vector3 targetPoint;

            public float overflow;
            public float prevOverflow;
            public float overflowVelocity;
            public float bottomOutForce;

            public bool bottomedOut;
            public bool overExtended;

            // ---------------------まとめて確認用。後で消すこと-----------------
            [Range(-15000.0f, 15000.0f)]
            public float carForce_x;
            [Range(-10000.0f, 20000.0f)]
            public float carForce_y;
            [Range(-15000.0f, 15000.0f)]
            public float carForce_z;
            // -----------------------------------------------------------------
            public float MaxLoad = 8000.0f;     // 荷重の最大値(0825_船渡)
            public float MinLoad = 2000.0f;        // 荷重の最小値(0825_船渡)
            public float LoadCorrection = 0.1f; // 荷重適用率

            public Vector3 BeforeForce;
            public float totalForce;
        }

        // 新しく追加したサスペンション処理で使用するクラス(0831_船渡)
        [System.Serializable]
        public class Suspension
        {
            [Range(-1000.0f, 3000.0f)]
            public float SuspensionForce;                   // サスペンションの力
            public float SuspensionHeight;                  // サスペンションの位置(0830_船渡)
            public Vector3 ForcePoint;                      // サスペンションの力を与える位置
            public float SpringConstant = 500.0f;           // ばね定数(0830_船渡)
            public float ExtensionDamperConstant = 3000.0f;   // スプリングが伸びている時のダンパーの減衰係数
            public float CompressionDamperConstant = 3500.0f; // スプリングが縮んでいる時のダンパーの減衰係数
            public float GroundPos;                         // 地面の座標

            public float DefaultLength = 0.5f;
            public float LoadCorrection = 0.2f;
        }

        [System.Serializable]
        public class Wheel
        {
            public float mass = 25.0f;
            public float rimOffset = 0f;
            public float tireRadius = 0.25f;
            public float width = 0.25f;
            public float dragForce = 50f;

            public float rpm;

            public Vector3 prevWorldPosition;
            public Vector3 worldPosition;
            public Vector3 prevGroundPoint;
            public Quaternion worldRotation;

            public AnimationCurve camberCurve = null;
            public float camberAngle;

            public float inertia;

            public float angularVelocity;
            public float freeRollingAngularVelocity;
            public float residualAngularVelocity;
            public float steerAngle;
            public float rotationAngle;
            public GameObject visual;
            public GameObject nonRotating;
            public GameObject rim;
            public Transform rimCollider;
            public bool isVisualNonSteer;

            public Vector3 up;
            public Vector3 inside;
            public Vector3 forward;
            public Vector3 right;
            public Vector3 velocity;
            public Vector3 prevVelocity;
            public Vector3 acceleration;

            public float tireLoad;

            public float motorTorque;
            public float brakeTorque;


            public Vector3 nonRotatingPostionOffset;

            public void Initialize(WheelController wc)
            {
                inertia = 0.5f * mass * (tireRadius * tireRadius + tireRadius * tireRadius) * 0.1f;

                rim = new GameObject();
                rim.name = "RimCollider";
                rim.transform.position = wc.transform.position + wc.transform.right * rimOffset * (int)wc.vehicleLRSide;
                rim.transform.parent = wc.transform;
                rim.layer = LayerMask.NameToLayer("Ignore Raycast");

                if (wc.useRimCollider && visual != null)
                {
                    MeshFilter mf = rim.AddComponent<MeshFilter>();
                    mf.name = "Rim Mesh Filter";
                    mf.mesh = wc.GenerateRimColliderMesh(visual.transform);
                    mf.mesh.name = "Rim Mesh";

                    MeshCollider mc = rim.AddComponent<MeshCollider>();
                    mc.name = "Rim MeshCollider";
                    mc.convex = true;

                    PhysicMaterial material = new PhysicMaterial();
                    material.staticFriction = 0f;
                    material.dynamicFriction = 0f;
                    material.bounciness = 0.3f;
                    mc.material = material;

                    wc.wheel.rimCollider = rim.transform;
                }
            }

            public void GenerateCamberCurve(float camberAtBottom, float camberAtTop)
            {
                AnimationCurve ac = new AnimationCurve();
                ac.AddKey(0.0f, camberAtBottom);
                ac.AddKey(1.0f, camberAtTop);
                camberCurve = ac;
            }
        }
        [System.Serializable]
        public class WheelHit
        {
            [SerializeField]
            public RaycastHit raycastHit;
            public float angleForward;
            public float distanceFromTire;
            public Vector2 offset;

            [HideInInspector]
            public float weight;
            public bool valid = false;
            public float curvatureOffset;
            public Vector3 groundPoint;

            public WheelHit() { }

            public Vector3 point
            {
                get
                {
                    return groundPoint;
                }
            }

            public Vector3 normal
            {
                get
                {
                    return raycastHit.normal;
                }
            }
            public Vector3 forwardDir;
            public float forwardSlip;
            public Vector3 sidewaysDir;
            public float sidewaysSlip;
            public float force;
            public Collider collider
            {
                get
                {
                    return raycastHit.collider;
                }
            }
        }
        #endregion


        #region Functions
        /*****************************/
        /* FUNCTIONS                 */
        /*****************************/

        public void Initialize()
        {
            if (parent == null) parent = FindParent();
            if (wheel == null) wheel = new Wheel();
            if (spring == null) spring = new Spring();
            if (damper == null) damper = new Damper();
            if (fFriction == null) fFriction = new Friction();
            if (sFriction == null) sFriction = new Friction();

            if (springCurve == null || springCurve.keys.Length == 0) springCurve = GenerateDefaultSpringCurve();
            if (damperCurve == null || damperCurve.keys.Length == 0) damperCurve = GenerateDefaultDamperCurve();
            if (wheel.camberCurve == null || wheel.camberCurve.keys.Length == 0) wheel.GenerateCamberCurve(0, 0);
            if (activeFrictionPreset == null) activeFrictionPreset = FrictionPreset.Ice;

            if (vehicleLRSide == LRSide.Auto && parent != null) vehicleLRSide = DetermineSide(transform.position, parent.transform);
        }


        private GameObject FindParent()
        {
            Transform t = transform;
            while (t != null)
            {
                if (t.GetComponent<Rigidbody>())
                {
                    return t.gameObject;
                }
                else
                {
                    t = t.parent;
                }
            }
            return null;
        }


        private AnimationCurve GenerateDefaultSpringCurve()
        {
            AnimationCurve ac = new AnimationCurve();
            ac.AddKey(0.0f, 0.0f);
            ac.AddKey(1.0f, 1.0f);
            return ac;
        }


        private AnimationCurve GenerateDefaultDamperCurve()
        {
            AnimationCurve ac = new AnimationCurve();
            ac.AddKey(0f, 0f);
            ac.AddKey(100f, 400f);
            return ac;
        }


        public Mesh GenerateRimColliderMesh(Transform rt)
        {
            Mesh mesh = new Mesh();
            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();

            var halfWidth = wheel.width / 1.5f;
            float theta = 0.0f;
            float startAngleOffset = Mathf.PI / 18.0f;
            float x = tireRadius * 0.5f * Mathf.Cos(theta);
            float y = tireRadius * 0.5f * Mathf.Sin(theta);
            Vector3 pos = rt.InverseTransformPoint(wheel.worldPosition + wheel.up * y + wheel.forward * x);
            Vector3 newPos = pos;

            int vertexIndex = 0;
            for (theta = startAngleOffset; theta <= Mathf.PI * 2 + startAngleOffset; theta += Mathf.PI / 12.0f)
            {
                if (theta <= Mathf.PI - startAngleOffset)
                {
                    x = tireRadius * 1.1f * Mathf.Cos(theta);
                    y = tireRadius * 1.1f * Mathf.Sin(theta);
                }
                else
                {
                    x = tireRadius * 0.1f * Mathf.Cos(theta);
                    y = tireRadius * 0.1f * Mathf.Sin(theta);
                }

                newPos = rt.InverseTransformPoint(wheel.worldPosition + wheel.up * y + wheel.forward * x);

                // 左
                Vector3 p0 = pos - rt.InverseTransformDirection(wheel.right) * halfWidth;
                Vector3 p1 = newPos - rt.InverseTransformDirection(wheel.right) * halfWidth;

                // 右
                Vector3 p2 = pos + rt.InverseTransformDirection(wheel.right) * halfWidth;
                Vector3 p3 = newPos + rt.InverseTransformDirection(wheel.right) * halfWidth;

                vertices.Add(p0);
                vertices.Add(p1);
                vertices.Add(p2);
                vertices.Add(p3);

                triangles.Add(vertexIndex + 3);
                triangles.Add(vertexIndex + 1);
                triangles.Add(vertexIndex + 0);

                triangles.Add(vertexIndex + 0);
                triangles.Add(vertexIndex + 2);
                triangles.Add(vertexIndex + 3);

                pos = newPos;
                vertexIndex += 4;
            }

            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            return mesh;
        }

        private Vector3 Vector3Average(List<Vector3> vectors)
        {
            Vector3 sum = Vector3.zero;
            foreach (Vector3 v in vectors)
            {
                sum += v;
            }
            return sum / vectors.Count;
        }


        private float AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
        {
            return Mathf.Atan2(
              Vector3.Dot(n, Vector3.Cross(v1, v2)),
              Vector3.Dot(v1, v2)) * Mathf.Rad2Deg;
        }

        public LRSide DetermineSide(Vector3 pointPosition, Transform referenceTransform)
        {
            Vector3 relativePoint = referenceTransform.InverseTransformPoint(pointPosition);

            if (relativePoint.x < 0.0f)
            {
                return WheelController.LRSide.Left;
            }
            else
            {
                return WheelController.LRSide.Right;
            }
        }

        public static bool IsInLayerMask(int layer, LayerMask layermask)
        {
            return layermask == (layermask | (1 << layer));
        }
        #endregion

        #region PhysicsFunctions
        /*****************************/
        /* 2022/05/29作成            */
        /* 21cu0312 河合奏 追記      */
        /* 物理計算メソッド  　　　  */
        /*****************************/

        //車の推進力計算処理
        public Vector3 WheelToBodyAddForce
        {
            //計算結果を保存しながら返す
            get
            {
                //後輪だった場合
                if ((int)vehicleFRSide == -1)
                {
                    //推進力の計算
                    //参考：https://dskjal.com/car/car-physics-for-simulator.html#traction のトラクション計算式
                    return traction = (NomalForward * wheel.motorTorque * 2.8f / wheel.tireRadius);
                }
                //例外が発生した場合
                return Vector3.zero;
            }
        }

        //車輪の転がり抵抗の計算処理
        public Vector3 RollingResistance
        {
            //計算結果を保存しながら返す
            get
            {
                RAccel = (Mathf.Pow(sFriction.speed, 2.0f) / wheel.tireRadius);  //現在の横方向の加速度をタイヤの角速度の計算法で算出
                preRVelocity = sFriction.speed;                                 //次フレームで計算するために現在の横向きの速度を保存

                //車輪に対して後ろ向きの転がり抵抗を計算
                //参考：https://dskjal.com/car/car-physics-for-simulator.html#traction の転がり抵抗計算式
                rollingResistance = NomalForward * 0.015f * LoadForceDistribution;

                //荷重が０の場合、処理を飛ばす（処理軽減）
                if (LoadForceDistribution == 0.0f) return rollingResistance;

                //現在タイヤのアウトコース側にかかっている力に対する摩擦抵抗を計算
                //参考：http://www.carphys.net/tire/frictcircle.html
                float SideForce = RAccel * (LoadForceDistribution / Physics.gravity.magnitude);     //現在かかっている横方向の力
                float MaxStopForce = (MaxStopFriction * LoadForceDistribution);                     //静的摩擦力の最大値
                float NowRiht = -1.0f * Mathf.Sign(sFriction.speed);                                //現在働いている横方向の逆ベクトル符号

                //現在タイヤにかかっている力が最大静止摩擦力より小さい場合
                if (MaxStopForce > SideForce)
                {
                    //アウトコース側の力と同等の力を逆ベクトルで加える
                    rollingResistance += NomalRight                     //タイヤの右単位ベクトル
                                        * SideForce                     //現在の静的摩擦力
                                        * NowRiht;                      //現在の右方向の力の逆ベクトル

                    //ハンドルの重さを摩擦力の大きさ分付与する（初期の力の大きさを上げるために最低値を下げている）
                    EfectPower = (int)(Mathf.InverseLerp(-(MaxStopForce * 4.0f), (MaxStopForce), Mathf.Abs(SideForce)) * 50.0f);
                }
                else
                {
                    //動的摩擦力の計算
                    float MoveForce = (MoveFriction * LoadForceDistribution);

                    //それ以外の場合動的摩擦係数分の力を逆ベクトルで加える
                    rollingResistance += NomalRight                     //タイヤの右単位ベクトル
                                        * MoveForce                     //動的摩擦力
                                        * NowRiht;                      //現在の右方向の力の逆ベクトル

                    //ハンドルの重さを摩擦力の大きさ分付与する（スリップ時の摩擦力なので大げさに表現している）
                    EfectPower = (int)(Mathf.InverseLerp(0.0f, MaxStopForce, MoveForce) * 40.0f);
                }

                //計算結果を返す
                return rollingResistance;
            }
        }

        //車輪のコーナリングフォースの計算処理
        public Vector3 CorneringForce
        {
            //計算結果を保存しながら返す
            get
            {
                if(fFriction.speed > 1.0f) { }
                //スリップ角に対して過剰に反応して力を加えて車体の揺れが発生するため、コーナリングフォースを弱める係数を計算する
                //低スピード時で小さいスリップ角の時は弱める。
                float antiShakeCoef = Mathf.Abs(sFriction.speed) < 1.0f && Mathf.Abs(steerAngle) < 5f ? sFriction.speed * sFriction.speed : 1.0f;

                //直進方向に対しての横方向ノーマルベクトルへのコーナリングフォースの計算
                //参考：https://dskjal.com/car/car-physics-for-simulator.html#traction のコーナリングフォース計算式
                //コーナリングパワーについてはカーブを使用
                corneringforce = NomalRight                             //直進方向に対しての横方向ノーマルベクトル
                                    * Mathf.Sign(sFriction.slip) * corneringForceCurve.Evaluate(Mathf.Abs(sFriction.slip))  //コーナリングパワー
                                    * LoadForceDistribution                                                                 //荷重
                                    * CurveFactor                                                                           //カーブする強さの補正
                                    * antiShakeCoef;                                                                        //震え補正
                return corneringforce;
            }
        }

        //ブレーキ処理、改良予定につきノーコメントで
        // 追加河合
        // 2023/07/19
        public Vector3 BrakeForce
        {
            get
            {
                float BrakeFoce = 10.0f * vc.input.Brake;
                float fFoce = (fFriction.speed * (LoadForceDistribution / Physics.gravity.magnitude)) + traction.magnitude;

                if (0.95f * BrakeFoce > MathF.Abs(fFoce))
                {
                    //アウトコース側の力と同等の力を逆ベクトルで加える
                    return rollingResistance = NomalForward                                 //タイヤの右単位ベクトル
                                                * fFoce;                                       //最大静止摩擦係数
                }
                //それ以外の場合動的摩擦係数分の力を逆ベクトルで加える
                float TireAnglVelo = MathF.Abs(fFoce) - (0.85f * BrakeFoce);   //静的摩擦係数
                if ((fFriction.speed * (LoadForceDistribution / Physics.gravity.magnitude)) < TireAnglVelo)
                {
                    //アウトコース側の力と同等の力を逆ベクトルで加える
                    return rollingResistance = NomalForward                                 //タイヤの右単位ベクトル
                                                * (0.85f * BrakeFoce);      //静的摩擦係数
                }

                //アウトコース側の力と同等の力を逆ベクトルで加える
                return rollingResistance = NomalForward                                                       //タイヤの右単位ベクトル
                                            * ((0.85f * BrakeFoce)                                      //静的摩擦係数
                                            + (0.85f * LoadForceDistribution * vc.input.Brake));      //静的摩擦係数
            }
        }

        //荷重計算処理
        public float LoadForceDistribution
        {
            //計算結果を保存しながら返す
            get
            {
                //load.csで行った荷重分配計算を利用し、車輪にかかっている力の大きさを保存しながら返す
                return load = parentRigidbody.mass                                          //Rigidbodyの重さ
                            * vc.loadCom.LerpForceCoef(vehicleFRSide, vehicleLRSide)        //load.csで行った荷重分配比率
                            * Physics.gravity.magnitude;                                    //重力加速度
            }
        }

        // 前輪にかかる荷重計算処理
        public float FrontLoadForceDistribution
        {
            //計算結果を保存しながら返す
            get
            {
                float loadA = load = parentRigidbody.mass                                          //Rigidbodyの重さ
                            * vc.loadCom.LerpForceCoef(FRSide.Front, LRSide.Left);        //load.csで行った荷重分配比率

                float loadB = load = parentRigidbody.mass                                          //Rigidbodyの重さ
                            * vc.loadCom.LerpForceCoef(FRSide.Front, LRSide.Right);        //load.csで行った荷重分配比率

                float result = (loadA + loadB) * Physics.gravity.magnitude;                                    //重力加速度


                //load.csで行った荷重分配計算を利用し、車輪にかかっている力の大きさを保存しながら返す
                return result;
            }
        }


        // 後輪にかかる荷重計算処理
        public float BackLoadForceDistribution
        {
            //計算結果を保存しながら返す
            get
            {
                float loadA = load = parentRigidbody.mass                                          //Rigidbodyの重さ
                            * vc.loadCom.LerpForceCoef(FRSide.Rear, LRSide.Left);        //load.csで行った荷重分配比率

                float loadB = load = parentRigidbody.mass                                          //Rigidbodyの重さ
                            * vc.loadCom.LerpForceCoef(FRSide.Rear, LRSide.Right);        //load.csで行った荷重分配比率


                float result = (loadA + loadB) * Physics.gravity.magnitude;                                    //重力加速度


                //load.csで行った荷重分配計算を利用し、車輪にかかっている力の大きさを保存しながら返す
                return result;
            }
        }
        public Vector3 vcForcePoint
        {
            get
            {
                return forcePoint;
            }
        }

        public Vector3 vcForce
        {
            get
            {
                return totalForce;
            }
        }
        #endregion
    }
}