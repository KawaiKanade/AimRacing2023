//────────────────────────────────────────────
// ファイル名	：Load.cs
// 概要			：荷重移動の計算を行い、タイヤに影響を与える機能の実装
// 作成者		：東樹 潤弥
// 作成日		：2020.3.05
// 
//────────────────────────────────────────────
// 更新履歴：
// 2020/03/05 [東樹 潤弥] クラス作成
// 2020/03/06 [東樹 潤弥] ダウンフォースの影響を追加
// 2020/03/07 [東樹 潤弥] 補正値の計算方法を変更
// 2020/03/16 [東樹 潤弥] 加速度の計算を追加
// 2020/03/18 [東樹 潤弥] 旋回半径の計算を追加中
// 2020/03/19 [東樹 潤弥] 旋回半径の計算を追加完了
// 2020/04/13 [東樹 潤弥] 荷重計算編集中
// 2020/04/15 [東樹 潤弥] 荷重前後左右ともに割合計算
// 2020/07/01 [東樹 潤弥] radRollの計算を車体から、タイヤの角度に変更
//────────────────────────────────────────────

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AIM
{
    // 荷重移動などを計算する
    public class Load : MonoBehaviour
    {
        private VehicleController vc;
        public bool LoadOn;

        [SerializeField]
        private WheelController wheelController_FL;
        [SerializeField]
        private WheelController wheelController_FR;
        [SerializeField]
        private WheelController wheelController_RL;
        [SerializeField]
        private WheelController wheelController_RR;

        [Header("ダウンフォースで補正される値の最大値")]
        public float adjustCoefValueMax = 0.2f;

        public Downforce downforce;
        private Downforce.DownforcePoint downforcePoint_F;
        private Downforce.DownforcePoint downforcePoint_R;

        // WheelControlerに渡す値
        public float forceCoef_FL;
        public float forceCoef_FR;
        public float forceCoef_RL;
        public float forceCoef_RR;

        public float forceCoef_F;
        public float forceCoef_R;

        // ----------------------椅子の移動で使用する値------------------
        public float forceCoef_Front;
        public float forceCoef_Rear;

        public float forceCoef_Left;
        public float forceCoef_Right;

        public float GetFrontLoad
        {
            get { return forceCoef_Front; }

        }

        public float GetRearLoad
        {
            get { return forceCoef_Rear; }

        }
        public float GetLeftLoad
        {
            get { return forceCoef_Left; }
        }

        public float GetRightLoad
        {
            get { return forceCoef_Right; }
        }
        // ----------------------------------------------------
        Rigidbody rigidbody;
        float acceleration;
        public float turningRadius;

        float speed = 0.0f;
        float prevSpeed = 0.0f;
        float time = 0.0f;

        float wheelBase;
        float treadWidth;

        // 重心からタイヤまでの距離
        float lengthFront;   // 重心から前輪までの距離
        float lengthRear;    // 重心から後輪までの距離
                             //----------------------------------------------------------
                             //2023/06/25 河合追記
        Vector3 lengthFR;           // 重心から前輪右側までの距離
        Vector3 lengthFL;           // 重心から前輪左側までの距離
        Vector3 lengthRR;           // 重心から後輪右側までの距離
        Vector3 lengthRL;           // 重心から後輪右側までの距離

        public Vector3 CenterOfMass;

        public Vector2 RollCenterLength;    // ロールセンターから重心までの距離
        public Vector3 RollCenter;         // ロールセンターの位置
        float RollAngle;            // ロールの角度
        public Vector3 BalancePoint;            //重心
                                                //----------------------------------------------------------
        float heightCenter;
        float radRoll;
        private const float closeToZero = 0.01f;

        //======================================
        [Header("=============================================")]
        LinearFunction linearFunction = new LinearFunction();

        LinearFunction.AxisParallel frontAxis;
        LinearFunction.AxisParallel rearAxis;
        Vector2 intersection;

        private Transform[] wheelTrans = new Transform[2];

        bool isRight;
        bool isntCurve;

        // front同士の交点の距離が、frontwheelとrearwheelの交点との距離より長くなる閾値（内輪の角度）
        // 滑りを考慮して、交点をfront同士の物に切り替える
        public const float intersectionSwitchRad = 16.0f;

        // 動的分配比率
        float distributionRatio_F;
        float distributionRatio_R;
        float distributionRatio_O;
        float distributionRatio_I;

        // 荷重
        /*[HideInInspector]*/
        public float load_FL;
        /*[HideInInspector]*/
        public float load_FR;
        /*[HideInInspector]*/
        public float load_RL;
        /*[HideInInspector]*/
        public float load_RR;

        public float rollCoeff;
        public float lerpSpeed;

        [SerializeField] private float rearDriftLoad;

        //============================================================================================================

        private void Start()
        {
            vc = GetComponent<VehicleController>();
            if (downforce != null)
            {
                downforcePoint_F = downforce.downforcePoints[0];
                downforcePoint_R = downforce.downforcePoints[1];
            }
            rigidbody = GetComponent<Rigidbody>();

            // トレッド、ホイールベースを計算
            CalcWheelParameters();
        }
        //============================================================================================================
        private void Update()
        {
            // 加速度を計算
            CalcAcceleration();

            // 旋回半径を計算
            turningRadius = CalcTurningRadius();

            //重心周りの計算
            CenteOfMassLength();

            // 荷重を計算
            CalcLoad();

            if (LoadOn)
            {
                MoveLoad();
            }
        }
        //============================================================================================================
        // 荷重の移動を計算し、修正値を返す
        public void MoveLoad()
        {
            float totalLoad = load_FL + load_FR + load_RL + load_RR;

            forceCoef_FL = (load_FL / totalLoad);
            forceCoef_FR = (load_FR / totalLoad);
            forceCoef_RL = (load_RL / totalLoad);
            forceCoef_RR = (load_RR / totalLoad);

            forceCoef_F = forceCoef_FL + forceCoef_FR;
            forceCoef_R = forceCoef_RL + forceCoef_RR;

            // --------------------------------------------------
            //前後の荷重
            forceCoef_Front = forceCoef_FL + forceCoef_FR;
            forceCoef_Rear = forceCoef_RL + forceCoef_RR;
            //左右の荷重
            forceCoef_Left = forceCoef_FL + forceCoef_RL;
            forceCoef_Right = forceCoef_FR + forceCoef_RR;
            // ---------------------------------------------------
        }
        //============================================================================================================
        public float GetForceCoef(WheelController.FRSide frSide, WheelController.LRSide lrSide)
        {
            if (frSide == WheelController.FRSide.Front)
            {
                if (lrSide == WheelController.LRSide.Right) { return forceCoef_FR; }
                else if (lrSide == WheelController.LRSide.Left) { return forceCoef_FL; }
            }
            else if (frSide == WheelController.FRSide.Rear)
            {
                if (lrSide == WheelController.LRSide.Right) { return forceCoef_RR; }
                else if (lrSide == WheelController.LRSide.Left) { return forceCoef_RL; }
            }
            return 0.0f;
        }
        //============================================================================================================
        // 加速度を計算する
        void CalcAcceleration()
        {
            // 加速度をどう計算するか
            // a = (v2 - v1)/(t2 - t1)
            // T1とV1の更新を1秒毎にして、T2,V2を毎フレーム更新する？
            // T1は0として、T2はTime.deltaTimeにする。V1は前フレームの速度、V2は現在速度にする？ <= 採用 <=　は？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？

            speed = transform.InverseTransformDirection(rigidbody.velocity).z;
            time = Time.deltaTime;

            acceleration = (speed - prevSpeed) / time;

            prevSpeed = speed;
        }
        //============================================================================================================
        // 旋回半径を計算するメソッド
        // 参考URL
        // http://hamanako-kankou.com/turedure/abs/abs.html
        float CalcTurningRadius()
        {
            float MinAngle;

            if (wheelController_FL.steerAngle < 0.0f)
            { isRight = false; }
            else if (wheelController_FL.steerAngle > 0.0f)
            { isRight = true; }

            // 角度がついていない場合、計算しない
            else { return 0.0f; }

            // 旋回半径に使うタイヤの設定
            Vector3 front = Vector3.zero;
            if (isRight)
            {
                MinAngle = wheelController_FR.steerAngle;
            }
            else
            {
                MinAngle = wheelController_FL.steerAngle;
            }

            return wheelBase / Mathf.Sin(MinAngle * Mathf.Deg2Rad);
        }
        //============================================================================================================
        // 最初に車のパラメータを計算する
        void CalcWheelParameters()
        {
            float preBaseL = wheelController_FL.transform.localPosition.z - wheelController_RL.transform.localPosition.z;
            float preBaseR = wheelController_FR.transform.localPosition.z - wheelController_RR.transform.localPosition.z;

            wheelBase = Mathf.Abs((preBaseL + preBaseR) / 2.0f);

            float preTreadL = wheelController_FL.transform.localPosition.x - wheelController_FR.transform.localPosition.x;
            float preTreadR = wheelController_RL.transform.localPosition.x - wheelController_RR.transform.localPosition.x;

            treadWidth = Mathf.Abs((preTreadL + preTreadR) / 2.0f);
        }
        //============================================================================================================
        void CalcLoad()
        {
            // 回転角度がマイナスにならないので、左だけが大きくなっている
            radRoll = Mathf.Deg2Rad * vc.steering.Angle * rollCoeff;
            //radRoll  = Mathf.Deg2Rad * transform.localEulerAngles.y;

            // 荷重計算（前後）
            distributionRatio_F = rigidbody.mass * (lengthRear / wheelBase) - (heightCenter / wheelBase) * (rigidbody.mass / 9.8f) * acceleration;
            distributionRatio_R = rigidbody.mass * (lengthFront / wheelBase) + (heightCenter / wheelBase) * (rigidbody.mass / 9.8f) * acceleration;

            ////荷重計算（左右）
            //if (turningRadius > 0.0f)
            //{
            //    distributionRatio_O = 0.5f - (heightCenter / treadWidth) * (radRoll - (1.0f / 9.8f) * ((speed * speed) / turningRadius));
            //    distributionRatio_I = 0.5f + (heightCenter / treadWidth) * (radRoll - (1.0f / 9.8f) * ((speed * speed) / turningRadius));
            //}
            //else
            //{
            //    distributionRatio_O = 0.5f;
            //    distributionRatio_I = 0.5f;
            //}

            //if (turningRadius != 0.0f)
            //{
            //    if (isRight)
            //    {
            //        distributionRatio_O = (rigidbody.mass * (lengthFR.magnitude + RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) + CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
            //        distributionRatio_I = (rigidbody.mass * (lengthFL.magnitude - RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) - CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
            //    }
            //    else
            //    {
            //        distributionRatio_O = (rigidbody.mass * (lengthFL.magnitude + RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) + CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
            //        distributionRatio_I = (rigidbody.mass * (lengthFR.magnitude - RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) - CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
            //    }
            //}
            //else
            //{
            //    distributionRatio_O = 0.5f;
            //    distributionRatio_I = 0.5f;
            //}

            // 松下変更---------------------------------------------------------
            //if (isRight)
            //{
            //	load_FL = (distributionRatio_F * distributionRatio_O);
            //	load_FR = (distributionRatio_F * distributionRatio_I);
            //	load_RL = (distributionRatio_R * distributionRatio_O);
            //	load_RR = (distributionRatio_R * distributionRatio_I);
            //}
            //else
            //{
            //	load_FL = (distributionRatio_F * distributionRatio_I);
            //	load_FR = (distributionRatio_F * distributionRatio_O);
            //	load_RL = (distributionRatio_R * distributionRatio_I);
            //	load_RR = (distributionRatio_R * distributionRatio_O);
            //}

            if (/*turningRadius != 0.0f*/true)
            {
                if (true)
                {
                    load_FL = distributionRatio_F * ((-(BalancePoint.x) + 5.0f) / 10.0f);
                    load_FR = distributionRatio_F * (((BalancePoint.x) + 5.0f) / 10.0f);
                    load_RL = distributionRatio_R * ((-(BalancePoint.x) + 5.0f) / 10.0f);
                    load_RR = distributionRatio_R * (((BalancePoint.x) + 5.0f) / 10.0f);
                }
                else
                {
                    load_FL = (distributionRatio_F * (lengthFR.magnitude - RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) - CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
                    load_FR = (distributionRatio_F * (lengthFL.magnitude + RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) + CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
                    load_RL = (distributionRatio_R * (lengthFR.magnitude - RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) - CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
                    load_RR = (distributionRatio_R * (lengthFL.magnitude + RollCenterLength.magnitude * Mathf.Tan(RollAngle * Mathf.Deg2Rad)) + CentrifugalForce * RollCenterLength.magnitude) / treadWidth;
                }
            }
            else
            {
                load_FL = distributionRatio_F / 2.0f;
                load_FR = distributionRatio_F / 2.0f;
                load_RL = distributionRatio_R / 2.0f;
                load_RR = distributionRatio_R / 2.0f;
            }
        }
        //=================================================================================
        // Loadの後輪反映処理
        public float loadRear()
        {
            if (forceCoef_R < rearDriftLoad)
            {
                return 1.0f - (rearDriftLoad - forceCoef_R);
            }
            return 1.0f;
        }
        //=================================================================================
        // Loadの後輪反映処理
        public float LerpForceCoef(WheelController.FRSide frSide)
        {
            float returnNum = 0.0f;
            if (frSide == WheelController.FRSide.Front)
            {
                returnNum = Mathf.Lerp(forceCoef_F, 1.0f, lerpSpeed);
            }
            else if (frSide == WheelController.FRSide.Rear)
            {
                returnNum = Mathf.Lerp(forceCoef_R, 1.0f, lerpSpeed);
            }
            return Mathf.Abs(returnNum);
        }

        public float LerpForceCoef(WheelController.FRSide frSide, WheelController.LRSide lrSide)
        {
            float returnNum = 0.0f;
            if (frSide == WheelController.FRSide.Front)
            {
                if (lrSide == WheelController.LRSide.Right) { returnNum = Mathf.Lerp(forceCoef_FR, 0.25f, lerpSpeed * Time.deltaTime)/*load_FL*/; }
                else if (lrSide == WheelController.LRSide.Left) { returnNum = Mathf.Lerp(forceCoef_FL, 0.25f, lerpSpeed * Time.deltaTime)/*load_FR*/; }
            }
            else if (frSide == WheelController.FRSide.Rear)
            {
                if (lrSide == WheelController.LRSide.Right) { returnNum = Mathf.Lerp(forceCoef_RR, 0.25f, lerpSpeed * Time.deltaTime)/*load_RL*/; }
                else if (lrSide == WheelController.LRSide.Left) { returnNum = Mathf.Lerp(forceCoef_RL, 0.25f, lerpSpeed * Time.deltaTime)/*load_RR*/; }
            }
            returnNum = Mathf.Clamp(returnNum, 0.0f, Mathf.Infinity);
            return Mathf.Abs(returnNum);
        }

        //----------------------------------------------------------
        //2023/06/25 河合変更＆追記
        //重心周りの距離を計算する関数
        void CenteOfMassLength()
        {
            //重心の位置を車のサイズに補正
            Vector3 CarBalance;
            CarBalance.x = (BalancePoint.x / 80.0f);
            CarBalance.y = /*(BalancePoint.y / 50.0f)*/0.0f;
            CarBalance.z = (BalancePoint.z / 40.0f);

            // 重心からタイヤまでの距離を計算
            lengthFront = Mathf.Abs(wheelController_FR.transform.localPosition.z - CarBalance.z);   //前輪
            lengthRear = Mathf.Abs(wheelController_RR.transform.localPosition.z - CarBalance.z);    //後輪
            lengthFR = wheelController_FR.transform.localPosition - CarBalance;                     //右前
            lengthFL = wheelController_FL.transform.localPosition - CarBalance;                     //左前
            lengthRR = wheelController_RR.transform.localPosition - CarBalance;                     //右後
            lengthRL = wheelController_RL.transform.localPosition - CarBalance;                     //左後

            // 地面から重心までの距離を計算
            heightCenter = Mathf.Abs(CarBalance.y - (wheelController_FL.Visual.transform.localPosition.y - wheelController_FL.radius));

            //ロールセンターから重心までの距離の計算
            RollCenter.y = wheelController_FL.Visual.transform.localPosition.y - (wheelController_FL.radius * 2.0f);        //ロールセンターの位置を設定（大体の予測で、根拠はない）
            RollCenterLength = CarBalance - RollCenter;                                             //ロールセンターから重心までの距離の計算

            //ロールセンターから見た重心の角度
            RollAngle = Vector2.Angle(RollCenterLength, Vector2.up);

            //重心位置の確認
            CenterOfMass = CarBalance;
        }

        //遠心力を計算する関数
        float CentrifugalForce
        {
            get
            {
                return (rigidbody.mass / 9.8f) * ((rigidbody.velocity.magnitude * rigidbody.velocity.magnitude) / turningRadius);

            }
        }

        //----------------------------------------------------------
    }

}