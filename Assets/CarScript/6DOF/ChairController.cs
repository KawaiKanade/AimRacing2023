
/*------------------------------------------------------------------
* ファイル名：ChairController.cs
* 概要：椅子をコントロール用のクラス
* 担当者：ゴコケン
* 作成日：2022/07/15
-------------------------------------------------------------------*/

/*----------------------------------------------------------------
更新者：河合奏/2023/09/04
更新内容：椅子に反映する回転計算すべてを変更。
          pitch、surge、roll、swayを車体の荷重、トランスミッションに依存
　　　　　yawをスリップ角に依存
          heaveをエンジンに依存するように変更。
 ----------------------------------------------------------------*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using AIM;

public class ChairController : MonoBehaviour
{
#if UNITY_EDITOR
    public bool showGraph = true;   // エディタ用
#endif
    //━━━━━━━━━━━━━━━━━━━━━━━━━━
    //              　　　プロパティ
    //━━━━━━━━━━━━━━━━━━━━━━━━━━
    #region 参照するクラス
    private Rigidbody rootRigidbody;                        // 車の鋼体
    public BalancePoint balancePoint { get; private set; }  // 重心のクラス
    public WIZMOController controller { get; private set; } // 椅子を制御するクラス
    private GoalJudgment goal;                              // ゴールしたかを取得用
    private GameObject transform1;                          // 車の座標を取得用
    private AIM.WheelInput input;                           // 入力取得用
    public Load load;                                   // 荷重取得用

    // ---------新たに使用するクラスの保存用(船渡)------------------
    public GameObject RootObject;                           // 親オブジェクトの保存
    public VehicleController VC;                            // 車体の保存
    private Transmission transmission;                      // トランスミッションの保存
    private Engine engine;                                  // エンジンの保存
    private bool FirstCall = true;							// 最初に取得する用

    public GameObject StartAccel;
    private ForcedOnAccel onAccel;
    private PlayStartFlag StartFlag;
    private bool GameStartFlag = false;
    // -------------------------------------------------------------
    #endregion

    #region 椅子の動きを調整する用
    public float farwordCoefficient = 1;                    // 前後の動きを調整用の係数
    public float rightCoefficient = 1;                      // 左右の動きを調整用の係数
    public float upCoefficient = 1;                         // 上下の動きを調整用の係数

    public float accelCoefficient = 1;                      //アクセルの調整用係数？
    public float brakeCoefficient = 0.5f;                   //ブレーキの調整用係数？
    public float shiftUpCoefficient = 1;                    //シフトチェンジ時の係数？
    public float shiftDownCoefficient = 1;                  //同上

    public Vector3 speedCoefficient = new Vector3(1, 1, 1);

    public float maxRotate = 1;                             // 椅子の最大回転量
    public Vector2 impactCoefficient = new Vector2(1, 1);   // ぶつかった後の衝撃を調整用の係数
    public float OverallTravelFactor = 0.5f;                // 全体の移動量を調整用の係数
    public float OverallSpeedFactor = 1;                      // 速度を調整用の係数

    //--------------どの程度値を適用するかどうかの倍率-------------
    public float RollCoefficient = 1.0f;
    public float SwayCoefficient = 1.0f;
    public float PicthCoefficient = 1.0f;
    public float SurgeCoefficient = 1.0f;
    public float YawCoefficient = 1.0f;
    private float HeaveCoefficient = 1.0f;
    public float RanHeaveCoefficient = 0.6f;
    public float StopHeaveCoefficient = 0.2f;
    public float ShiftCoefficient_AT = 0.5f;
    public float ShiftCoefficient_MT = 1.0f;
    //---------------------------------------------------------------
    public float SlipClamp = 30.0f;									// スリップ角の補正
    private float ForwardSpeed = 0.0f;                              // 前方への速度
    public float ChairNormalSpeed = 0.25f;                          // 通常時の速度
    public float Frequency = 1.5f;                                  // 振動の周波数
    public float CarRollValue = 0.2f;                               // 車が曲がっていると判別する値
    public float UpdateRollThreshold = 0.1f;                        // Rollの値を更新する値
    public float UpdatePicthThreshold = 0.1f;                       // pitchの値を更新する値
    public float UpdateSurgeThreshold = 0.05f;                      // surgeの値を更新する値
    public float MaxSurgeVariation = 0.15f;
    private float BeforeRoll = 0.0f;                                 // 前回のRollの値
    private float BeforePicth = 0.0f;                                // 前回のpitchの値
    private float BeforeSurge = 0.0f;
    private float BeforeYaw = 0.0f;
    private bool CarStart = false;
    public float MaxFrontPicth = -0.2f;                             // 前方pitchの最大値
    public float MaxRearPicth = 0.4f;                               // 後方Picthの最大値
    public float AddPicth = 0.1f;
    private float BeforeSpeed = 0.0f;                               // 前回の速度
    public float carSpeedMin = 0.05f;
    public float YawChangeMinSpeed = 5.0f;                          //Yawの変更を入れる最低速度 
    public float carAcceleration = 0.0f;
    private bool PitchFirstStop = true;
    public float MaxReduce = 1.0f;
    public float StoppingLoad = 0.0f;
    public float prevTime = 0.0f;
    private bool SurgeUpdate = false;
    public float MovePitchValue = 0.2f;
    public float MaxPitch = 0.2f;
    private bool BeforePicthFlag = false;

    public float DefoSpeed = 1.0f;
    public float MaxSpeed = 10.0f;
    public float MinSpeed = 0.5f;
    public float MaxfAccel = 10.0f;
    public float MinfAccel = 0.5f;
    public float DefoAccel = 0.8f;
    public float MinAccel = 0.4f;
    public float PreHeave = 0.0f;

    private bool GoalEnd = false;
    private bool GoalChairSet = false;
    public float GoalSpeed_ps = 0.7f;
    public float GoalAccel_ps = 0.5f;
    public float GoalSpeed = 0.5f;
    public float GoalAccel = 0.1f;

    private int NowGear = 0;
    private int BeforeGear = 0;

    private bool ShiftChange = false;
    private bool BeforeShiftChange = false;
    public float ShiftMaxSpeed = 20.0f;
    public float ShiftMaxScale = 1.0f;
    public float ShittPitch = 0.2f;
    private float ResultStorage = 0.0f;

    public float ShiftBackCount = 3.0f;
    private float ShiftBackCounter = 0.0f;
    #endregion

    #region デバッグ用
    public bool bIsConnected { get; private set; } = false;         // 椅子に接続したかのフラグ
    private int reconnectCnt = 0;                                   // 再接続までのカウンター
    public bool bEmergencyShutdown { get; private set; } = false;   // 緊急停止中かどうか
    #endregion

    #region フレームをカウント用
    public int zAxisCnt { get; private set; }   // ｚ軸の移動したフレーム数を保存用
    public int zAxisThreshold = 3;              // ｚ軸の移動したフレーム数の閾値
    // ------------------------------------
    public int FreamCount_Roll = 0;                    // Rollの経過フレームカウント用
    public int FreamCount_Pitch = 0;                   // Pitchの経過フレームカウント用
    public int FreamCount_Yaw = 0;                     // Yawの経過フレームカウント用
    public int ChangeCount_Roll = 60;               // Rollの移動目的地更新フレームの設定用
    public int ChangeCount_Pitch = 60;              // Pitchの移動目的地更新フレームの設定用
    public int ChangeCount_Yaw = 60;                // Yawの移動目的地更新フレームの設定用
    public bool CallChange_Roll = false;                    // Rollの目的地を更新するかどうか
    public bool CallChange_Pitch = false;                   // Pitchの目的地を更新するかどうか
    public bool CallChange_Yaw = false;                     // Yawの目的地を更新するかどうか
    // ------------------------------------
    public bool CallRoll = true;
    public bool CallSway = true;
    public bool CallPitch = true;
    public bool CallSurge = true;
    public bool CallYaw = true;
    public bool CallHeave = true;
    public bool CallWall = true;
    #endregion

    #region 補間用
    public bool bBalancePointMoved_x { get; private set; }     // ｘ軸移動したかのフラグ
    public bool bBalancePointMoved_y { get; private set; }     // ｙ軸移動したかのフラグ
    public bool bBalancePointMoved_z { get; private set; }    // ｚ軸移動したかのフラグ
    public Vector3 movePercent = new Vector3(70, 70, 70);     // 毎回移動するときのパーセント
    public Vector3 returnPercent = new Vector3(20, 20, 20); // 戻るときのパーセント
    private Vector3 targetPos = Vector3.zero;                 // 移動先の「座標」
#if UNITY_EDITOR
    public Vector3 _targetPos { get { return targetPos; } }   // エディタに表示する用
#endif
    #endregion

    #region 振動
    public float RollingRandomRange = 0.05f;                  // 振動のランダム範囲
    public int RollingCnt { get; private set; } = 0;          // 振動の間隔をカウントする用
    public AnimationCurve rollingCurve;                       // 振動の間隔を決めるグラフ
    public Vector3 RollingCoefficient = new Vector3(1, 1, 1); // 振動の激しさを調整する用の係数
    #endregion

    #region ぶつかった後の処理用
    public bool bGotHit { get; private set; } = false;          // ヒットしたかのフラグ
    public int hitCnt { get; private set; } = 0;                // 処理の時間を計算用カウンター
    public int hitCntThreshold = 40;                            // 処理にかけるフレーム数を決める変数
    public float hitSpeed { get; private set; }                 // ヒットした後椅子の速度を保存用
    public float pitchAfterHit { get; private set; }            // ヒットした後ピッチの値を保存用
    public float rollAfterHit { get; private set; }             // ヒットした後ロールの値を保存用
    public float yawAfterHit { get; private set; }              // ヒットした後ヨーの値を保存用
    public AnimationCurve hitImpactCurve;                       // ヒットする時の衝撃を決めるグラフ
    public Vector2 returnPercentAfterHit = new Vector2(50, 50); // 元の位置に戻すパーセント
    public float accelerationAfterHit = 0.8f;                   // ヒットした後椅子の加速度
    public float maxSpeedAfterHit = 0.6f;                       // ヒットした後椅子の最大速度
    private bool bLR;                                           //ヒットしたのは左右のどちらの壁か
    private float saveHitMove;                                  //ヒット時の移動量保存用
    #endregion

    #region 地面の凹凸
    /*----------------------------------------------------------------
	作成日：2023/08/22
	担当：21cu0203_池田柊太
	概要：地面の凹凸による椅子の揺れを作成するための変数追加
	 ----------------------------------------------------------------*/

    public Vector3 UnevennessRoad = new Vector3(0, 0, 0);       //地面の凹凸情報
    public float MaxRange = 0.5f;                               //凹凸の最大値(Clampで使用)
    public float MinRange = -0.5f;                              //凹凸の最小値(Clampで使用)
    public float FreamCounter;                                  //フレームカウント用
    public bool bUsed = false;                                  //一度だけ処理をしたい時に使用
    public bool bOnDeceleration = false;                        //減速帯(継続してるやつ)の上に乗っているか
    public bool bUpDownDeceleration = false;                    //減速帯(交互に変わるやつ)に乗っているか

    #endregion


    //━━━━━━━━━━━━━━━━━━━━━━━━━━
    //              　　　メソッド
    //━━━━━━━━━━━━━━━━━━━━━━━━━━

    //━━━━━━━━━━━━━━━━━━━━━
    // ゲームの最初に呼ばれる関数
    //━━━━━━━━━━━━━━━━━━━━━
    void Start()
    {
        // 初期化
        Init();

        //2023坂本郁樹追記-----------------------------
        //椅子がシーンをまたいでも破棄されないようにする。
        DontDestroyOnLoad(controller);
        //--------------------------------------------

    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 毎フレームに呼ばれる関数
    //━━━━━━━━━━━━━━━━━━━━━
    void Update()
    {
        // -----------必要なスクリプト取得(2023/09/05船渡_0912追記)------------
        if (FirstCall)
        {
            // Startで取得しようとすると呼び出し順が不定の関係で
            // VehicleControllerにLoadがアタッチされる前に呼ばれる可能性があるため、こちらで処理
            load = VC.GetLoadClass;

            // 同上の理由で他も処理
            transmission = VC.GetTransmission;
            engine = VC.GetEngine;
            StartFlag = onAccel.GetPlayStartFlag;

            FirstCall = false;
        }
        // -----------------------------------------------------------
        // 今回のギア数を取得
        NowGear = transmission.Gear;
        // 接続のチェック
        #region if(!CheckChairConnect()) return;
#if !UNITY_EDITOR
		// 元の処理
		if(!CheckChairConnect()) return;
#else
        // デバッグ用
        CheckChairConnect();
#endif
        #endregion

        // 緊急停止キー
        if (!EmergencyShutdown()) return;

        // ゴールのチェック
        if (goal.getGoalFlag())
        {
            GoalRhairReset();
            if (GoalEnd)
            {
                // 椅子をリセットする
                //ResetChair();
                return;
            }
        }

        // OP再生が終わって、操作を受け付けるようになったら椅子のメイン処理を開始する
        if (StartFlag.GetFalg)
        {
            // 上下振動を適用するかの切り替え用
            if (Input.GetKeyDown(KeyCode.F10))
            {
                if (CallHeave)
                {
                    CallHeave = false;
                    controller.heave = 0.0f;
                }
                else
                {
                    CallHeave = true;
                }
            }

            // 今回のギア数を取得
            NowGear = transmission.Gear;

            // 値の更新
            //UpdateValueOfChair();
            if (!GoalEnd) UpdateChaieValue();

            // 今回のギアを保存
            BeforeGear = NowGear;

        }
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // オブジェクトが消される時に呼ばれる関数
    //━━━━━━━━━━━━━━━━━━━━━
    private void OnDestroy()
    {
        // 椅子の位置をリセット
        ResetChair();
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 初期化用の関数
    //━━━━━━━━━━━━━━━━━━━━━
    public void Init()
    {
        // 各フラグのリセット
        bEmergencyShutdown = false;
        bBalancePointMoved_x = false;
        bBalancePointMoved_y = false;
        bBalancePointMoved_z = false;

        // 参照するクラスを取得しておく
        balancePoint = transform.root.gameObject.transform.Find("carbody").GetComponent<BalancePoint>();
        controller = transform.gameObject.GetComponent<WIZMOController>();
        rootRigidbody = transform.root.GetComponent<Rigidbody>();
        transform1 = transform.root.transform.Find("carbody").Find("transform1").gameObject;
        goal = GameObject.Find("GoalGate").GetComponent<GoalJudgment>();
        input = transform.root.GetComponent<AIM.VehicleController>().wheelInput;
        // -------------------荷重やスリップ角を椅子の動きに適用するためのクラス取得(2023/09/05船渡)----------------
        RootObject = transform.root.gameObject;                                 // 親の車体
        VC = RootObject.GetComponent<AIM.VehicleController>();   // 車体情報
        // ---------------------------------------------------------------------------------------------------------
        // 開始状態を取得するためのクラス取得(2023/09/14船渡)
        StartAccel = GameObject.Find("ForcedOnAccel");
        onAccel = StartAccel.GetComponent<ForcedOnAccel>();

        // 椅子をリセット
        ResetChair();

        //2023坂本郁樹追記---------------
        //開始時の位置にセット
        if (CallYaw) controller.yaw = 0.0f;
        if (CallHeave) controller.heave = 0.5f;

        //椅子がゆっくり動くようにする
        controller.speedAxis123 = 0.1f;
        //------------------------------

        // 値の初期化
        targetPos = Vector3.zero;
        zAxisCnt = 0;

        // グラフの初期化
        // 振動の間隔
        rollingCurve.AddKey(0.5f, 30);
        rollingCurve.AddKey(3, 24);
        rollingCurve.AddKey(8, 24);
        rollingCurve.AddKey(35, 26);
        rollingCurve.AddKey(60, 30);
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 椅子をリセット用の関数
    //━━━━━━━━━━━━━━━━━━━━━
    public void ResetChair()
    {
        if (controller == null) return;
        // 速度
        controller.speedAxis123 = 1.0f;//0.4

        // 加速度
        controller.accelAxis123 = 1.0f;//0.25

        // 前後
        controller.pitch = 0;

        // 左右
        controller.roll = 0;
        controller.yaw = 0;

        // 上下
        controller.heave = 0;

        // ほかの使っていない変数
        controller.sway = 0;
        controller.surge = 0;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 椅子との接続をチェックする関数
    // 戻り値：使っていい場合はtrue、
    // 　　　　それ以外はfalse
    //━━━━━━━━━━━━━━━━━━━━━
    private bool CheckChairConnect()
    {
        // コントローラーが見つけない場合はこの後の処理をしない
        if (controller == null) return false;

        // 椅子が正常に動いているかをチェック
        bIsConnected = controller.isRunning();
        if (!bIsConnected)
        {
            // もし接続していない場合、
            // 600フレーム毎に接続してみる
            if (reconnectCnt++ >= 600)
            {
                // カウンターをリセット
                reconnectCnt = 0;

                // 再接続
                controller.OpenSIMVR();
                return false;
            }
            else return false;
        }
        return true;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 椅子の緊急停止の関数
    // 戻り値：使っていい場合はtrue、
    // 　　　　それ以外はfalse
    //━━━━━━━━━━━━━━━━━━━━━
    public bool EmergencyShutdown()
    {
        // 通常状態
        if (!bEmergencyShutdown)
        {
            // キーが押されたら、緊急停止します
            if (Input.GetKeyDown(KeyCode.F7))
            {
                bEmergencyShutdown = true;

                // 値を戻す
                ResetChair();

                controller.CloseSIMVR();
                return false;
            }
            return true;
        }
        // 緊急停止中
        else
        {
            // もう一度押すと元に戻す
            if (Input.GetKeyDown(KeyCode.F7))
            {
                bEmergencyShutdown = false;

                // 値を戻す
                controller.OpenSIMVR();
            }

            return false;
        }
    }

    // -----------------------------------------
    // 各種処理を行うかどうかのカウンターの確認
    // -----------------------------------------
    private void CountCheck()
    {
        // Rollの目的地更新フラグがオフの場合
        if (!CallChange_Roll)
        {
            // Rollの更新を管理するカウントを更新する
            ++FreamCount_Roll;

            // Rollの更新を管理するカウントが所定の値を超えたら
            if (FreamCount_Roll > ChangeCount_Roll)
            {
                // 更新処理を呼び出すフラグをオンにする
                CallChange_Roll = true;

                // カウンターをリセットする
                FreamCount_Roll = 0;
            }
        }

        // Pitchの目的地更新フラグがオフの場合
        if (!CallChange_Pitch)
        {
            // Pitchの更新を管理するカウントを更新する
            ++FreamCount_Pitch;

            // Pitchの更新を管理するカウントが所定の値を超えたら
            if (FreamCount_Pitch > ChangeCount_Pitch)
            {
                // 更新処理を呼び出すフラグをオンにする
                CallChange_Pitch = true;

                // カウンターをリセットする
                FreamCount_Pitch = 0;

                SurgeUpdate = true;
            }
        }

        // Yawの目的地更新フラグがオフの場合
        if (!CallChange_Yaw)
        {
            // Pitchの更新を管理するカウントを更新する
            ++FreamCount_Yaw;

            // Pitchの更新を管理するカウントが所定の値を超えたら
            if (FreamCount_Yaw > ChangeCount_Yaw)
            {
                // 更新処理を呼び出すフラグをオンにする
                CallChange_Yaw = true;

                // カウンターをリセットする
                FreamCount_Yaw = 0;
            }
        }
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 椅子の値を更新する関数
    //━━━━━━━━━━━━━━━━━━━━━
    private void UpdateValueOfChair()
    {
        //━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 優先順位で処理を分ける（１が一番高い）
        // １：ぶつかった後の処理
        // ２：シフトチェンジ・前後の処理　の最初のフレームの処理
        // ３：左右の処理（戻る処理を除く）
        // ４：シフトチェンジ・前後の処理　の続き（戻る処理を除く）
        // ５：前後・左右　の戻る処理
        // ６：振動・上下　の処理
        // 
        // ※５番と６番は毎フレームやる
        //━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        #region １：ぶつかった後の処理

        // ヒットした場合のみやる
        if (bGotHit)
        {
            // カウントをインクリメント
            ++hitCnt;

            float temp = DampedSinWave(hitCnt, hitCntThreshold / 2,
                  (_t) => { return Mathf.Pow(0.3f, Mathf.Sqrt(_t)); });



            //　カウントが(40/3)より小さい間は
            if (hitCnt < hitCntThreshold / 3)
            {
                //ヒット後の椅子の各回転の値をセット
                controller.pitch = pitchAfterHit;
                controller.roll = rollAfterHit;
                controller.yaw = yawAfterHit;

                //controller.accelAxis123 = 0.8f; //加速度

                //controller.speedAxis123 = Mathf.Clamp(hitSpeed * OverallSpeedFactor, 0, maxSpeedAfterHit);
            }
            // 戻すの処理
            else
            {
                // 前後の処理
                // アクセル踏んでいる場合
                if (input.inputAxel >= 0.01f ||
                    Input.GetKey(KeyCode.UpArrow) ||
                    Input.GetKey(KeyCode.W))
                {
                    if (Mathf.Abs(controller.pitch - 0.2f) > 0.05f)
                    {
                        controller.pitch = ((controller.pitch - 0.2f) * (100 - returnPercentAfterHit.y) / 100) + 0.2f;
                    }
                    else controller.pitch = 0.2f;
                }
                // 踏んでいない場合は普通に０に戻す
                else
                {
                    if (Mathf.Abs(controller.pitch) > 0.05f) controller.pitch *= (100 - returnPercentAfterHit.y) / 100;
                    else controller.pitch = 0;
                }

                // 左右の処理
                if (Mathf.Abs(controller.roll) > 0.05f) controller.roll *= (100 - returnPercentAfterHit.x) / 100;
                else controller.roll = 0;
                if (Mathf.Abs(controller.yaw) > 0.05f) controller.yaw *= (100 - returnPercentAfterHit.x) / 100;
                else controller.yaw = 0;

                // スピードをセットする
                controller.speedAxis123 = 0.25f;
            }

            if (hitCnt > hitCntThreshold)
            {
                // 値やフラグのリセット
                bGotHit = false;
                hitCnt = 0;
                controller.speedAxis123 = 0.4f;
                controller.accelAxis123 = 0.25f;
            }
            return;
        }

        #endregion

        #region ２：シフトチェンジ・前後の処理　の最初のフレームの処理

        //else if (bBalancePointMoved_z && zAxisCnt <= zAxisThreshold)
        //{
        //	++zAxisCnt;
        //	float temp = (targetPos.z - controller.pitch) * movePercent.z / 100;

        //	// スピードをセットする
        //	controller.speedAxis123 = Mathf.Clamp(Mathf.Abs(temp) * speedCoefficient.z * OverallSpeedFactor, 0.1f, 1);

        //	// 移動させる
        //	controller.pitch += temp;

        //	// ついたかをチェック
        //	if (Mathf.Abs(controller.pitch - targetPos.z) < 0.1f)
        //	{
        //		controller.pitch = targetPos.z;
        //		bBalancePointMoved_z = false;
        //	}
        //}

        #endregion

        #region ３：左右の処理（戻る処理を除く）
        //？X軸移動をしている　かつ　現在のスピードが５以上なら
        else if (bBalancePointMoved_x && balancePoint.forwardSpeed > 5)
        {
            //？Z軸移動を止める
            bBalancePointMoved_z = false;
            //？毎フレームの移動量を求めてる	(移動先のX座標 - コントローラーのroll) * 移動.xのパーセント / 100 ？
            float temp = (targetPos.x - controller.roll) * movePercent.x / 100;

            // スピードをセットする
            controller.speedAxis123 = Mathf.Clamp(Mathf.Abs(temp) * speedCoefficient.x * OverallSpeedFactor, 0.1f, 1);

            // 移動させる
            //？tempの結果をroll yaw　それぞれに加減する
            controller.roll += temp;
            controller.yaw -= temp;

            // ついたかをチェック
            //？（絶対値（椅子の左右回転 - 移動先のX座標）< 0.1	
            if (Mathf.Abs(controller.roll - targetPos.x) < 0.1f)
            {
                controller.roll = targetPos.x;  //？椅子の左右回転は移動先のX座標に
                controller.yaw = -targetPos.x; //？椅子の上下回転は移動先の-X座標に
                bBalancePointMoved_x = false;   //？X軸移動を止める
            }
        }

        #endregion

        #region ４：シフトチェンジ・前後の処理　の続き（戻る処理を除く）
        //？Z軸移動をしているなら？
        else if (bBalancePointMoved_z)
        {
            //？毎フレームの移動量を求めてる？　	(移動先のZ座標 - コントローラーのpitch) * 移動.Zのパーセント / 100 
            float temp = (targetPos.z - controller.pitch) * movePercent.z / 100;

            // スピードをセットする
            controller.speedAxis123 = Mathf.Clamp(Mathf.Abs(temp) * speedCoefficient.z * OverallSpeedFactor, 0.1f, 1);

            // 移動させる
            //？tempの結果を pitch に加算する
            controller.pitch += temp;

            // ついたかをチェック
            //？（絶対値（椅子の前後回転 - 移動先のZ座標）< 0.1	
            if (Mathf.Abs(controller.pitch - targetPos.z) < 0.1f)
            {
                controller.pitch = targetPos.z; //？椅子の前後を移動先のＺ座標に設定する
                bBalancePointMoved_z = false;   //？Ｚ軸移動を止める
            }
        }

        #endregion

        #region 他：１、２、３以外の場合

        else
        {
            // 速度と加速度をリセット
            controller.speedAxis123 = 0.4f;
            controller.accelAxis123 = 0.25f;
        }

        #endregion

        #region ５：前後・左右　の戻る処理

        // 値を戻す
        // 左右
        //？X軸移動していないなら　
        if (!bBalancePointMoved_x)
        {
            //？（絶対値（椅子の左右回転） ＞ 0.05f）なら　椅子の左右回転 *= （100 - 戻るときのパーセント.X） / 100　
            if (Mathf.Abs(controller.roll) > 0.05f) controller.roll *= (100 - returnPercent.x) / 100;
            else controller.roll = 0;   //？椅子の左右回転を０に（初期値）

            //？（絶対値（椅子の上下回転）＞ 0.05f）　なら　椅子の上下回転 *= （100 - 戻るときのパーセント.X） / 100　
            if (Mathf.Abs(controller.yaw) > 0.05f) controller.yaw *= (100 - returnPercent.x) / 100;
            else controller.yaw = 0;    //？椅子の上下回転を０に（初期値）
        }

        // 前後
        //？ブレーキの入力(ペダル)が0.01よりも大きい　又は　Sキー入力がある　又は↓入力があるなら
        if (input.inputBrake > 0.01f ||
                Input.GetKey(KeyCode.S) ||
                Input.GetKey(KeyCode.DownArrow))
        {
            // 速度が低い場合は真ん中の位置に戻す
            if (Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward) < 5)
            {
                controller.pitch = 0;
                controller.speedAxis123 = 0.5f;
                controller.accelAxis123 = 0.4f;
            }
        }
        //？Z軸移動をしていなければ
        else if (!bBalancePointMoved_z)
        {
            // アクセルを踏んでいる時は０じゃなくで０．２ｆに戻す
            //？アクセル入力(ペダル)が0.01f以上　又は　↑入力がある　又は　Wキー入力があるなら
            if (input.inputAxel >= 0.01f ||
                Input.GetKey(KeyCode.UpArrow) ||
                Input.GetKey(KeyCode.W))
            {
                //？（絶対値（椅子の前後回転 - 0.2f）< 0.05f	
                if (Mathf.Abs(controller.pitch - 0.2f) > 0.05f)
                {
                    //？椅子の前後回転　= ((椅子の前後回転 - 0.2f) * (100 - 戻るときのパーセント.Z) / 100) +0.2f
                    controller.pitch = ((controller.pitch - 0.2f) * (100 - returnPercent.z) / 100) + 0.2f;
                }
                else controller.pitch = 0.2f;   //？椅子の前後回転を0.2fに戻す　
            }
            // 踏んでいない場合は普通に０に戻す
            else
            {
                //？(絶対値(椅子の前後回転) > 0.05f)　なら　椅子の前後回転　*= (100 - 戻るときのパーセント.Z) / 100;
                if (Mathf.Abs(controller.pitch) > 0.05f) controller.pitch *= (100 - returnPercent.z) / 100;
                else controller.pitch = 0;  //？椅子の前後回転を０にする（初期値）
            }
        }

        #endregion

        #region ６：振動・上下　の処理
        float speed = Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward);

        //エンジンによる振動
        Rolling(speed);

        // 上下
        //？Y軸移動をしているなら
        if (bBalancePointMoved_y)
        {
            //？椅子の上下移動 += (移動先の座標.Y - 椅子の上下移動) * 移動するときのパーセント.Y / 100;
            controller.heave += (targetPos.y - controller.heave) * movePercent.y / 100;
            //？絶対値(椅子の上下移動 - 移動先の座標.Y)　が　0.1f　より小さいなら
            if (Mathf.Abs(controller.heave - targetPos.y) < 0.1f)
            {
                controller.heave = targetPos.y;     //？椅子の上下移動 = 移動先の座標.Y
                bBalancePointMoved_y = false;       //？Y軸移動を止める
            }
        }


        #endregion

        #region 他：範囲を超えないようにする
        //各回転が上限値を超えて回転しないように制限をかける
        controller.roll = Mathf.Clamp(controller.roll, -maxRotate, maxRotate);
        controller.pitch = Mathf.Clamp(controller.pitch, -maxRotate, maxRotate);
        controller.heave = Mathf.Clamp(controller.heave, -maxRotate, maxRotate);
        controller.yaw = Mathf.Clamp(controller.yaw, -maxRotate, maxRotate);
        controller.speedAxis123 = Mathf.Clamp(controller.speedAxis123, 0, 1);
        controller.accelAxis123 = Mathf.Clamp(controller.accelAxis123, 0, 1);

        #endregion

    }

    // -----------------------------------------
    // 椅子の値を更新する関数2023ver
    // -----------------------------------------
    private void UpdateChaieValue()
    {
        PreHeave = controller.heave;
        // 値を更新するかどうかの確認を行う
        CountCheck();

        // 車の速度を算出
        ForwardSpeed = Vector3.Dot(rootRigidbody.velocity, RootObject.transform.forward);

        // 車が走っている状態の場合
        if (CarStart)
        {
            // ブレーキが踏まれているかつ速度が最低速度より小さい場合
            if (VC.input.Brake > 0.0f && ForwardSpeed < MinSpeed)
            {
                // 車が止まっている状態にする
                CarStart = false;
            }
        }
        // そうでない場合
        else
        {
            // アクセルが押されているかつ車の速度が最低速度より大きい場合
            if (VC.input.Accel > 0.0f && ForwardSpeed > MinSpeed)
            {
                // 車が走っている状態にする
                CarStart = true;
            }
        }

        Debug.Log($"d_transmission.Gear : {transmission.Gear}");
        Debug.Log($"d_Engine : {engine.RPM}");

        #region １：ぶつかった後の処理

        // ヒットした場合のみやる
        if (bGotHit)
        {
            // カウントをインクリメント
            ++hitCnt;

            if (hitCnt > 10)
            {
                if (bLR)
                {
                    controller.roll += saveHitMove;
                    controller.sway -= saveHitMove;

                }
                else
                {
                    controller.roll -= saveHitMove;
                    controller.sway += saveHitMove;

                }
                hitCnt = 0;
                bGotHit = false;
                controller.speedAxis123 = 0.5f;
                controller.accelAxis123 = 0.5f;

            }
        }
        #endregion

        // Rollの更新を行う場合
        if (CallChange_Roll)
        {
            // 目標値の算出を行う
            float TargetValue = LoadSteering();

            // セットする二つの値に合わせて値の補正を行い
            float Roll = Mathf.Clamp(TargetValue * RollCoefficient, -maxRotate, maxRotate);
            float Sway = Mathf.Clamp(TargetValue * SwayCoefficient, -maxRotate, maxRotate);

            // 値をセットする
            if (CallRoll) controller.roll = Roll;
            if (CallSway) controller.sway = -Sway;

            // 更新フラグをオフにする
            CallChange_Roll = false;
        }

        // Pitchの更新を行う場合
        if (CallChange_Pitch)
        {
            // 目標値の更新を行う
            float TargetValue = PitchCalcTest();
            float TargetSurge = SurgeCalc();
            //PitchTest();
            // セットする二つの値に合わせて値の補正を行い
            float Pitch = Mathf.Clamp(TargetValue * PicthCoefficient, -maxRotate, maxRotate);
            float Surge = Mathf.Clamp(TargetSurge * SurgeCoefficient, -maxRotate, maxRotate);
            Debug.Log($"Surge : {Surge}");
            // 値をセットする
            if (CallPitch) controller.pitch = Pitch;
            if (CallSurge) controller.surge = Surge;

            // 更新フラグをオフにする
            CallChange_Pitch = false;
        }

        // Yawの更新を行う場合
        if (CallChange_Yaw)
        {
            // 目標値の更新を行う
            float TargetValue = YawValue();

            // セットする値に合わせて補正を行い
            float Yaw = Mathf.Clamp(TargetValue * YawCoefficient, -maxRotate, maxRotate);

            // 値をセットする
            if (CallYaw) controller.yaw = Yaw;

            // 更新フラグをオフにする
            CallChange_Yaw = false;
        }

        // Heaveの更新を行う場合
        if (CallHeave)
        {
            if (CarStart)
            {
                HeaveCoefficient = RanHeaveCoefficient;
            }
            else
            {
                HeaveCoefficient = StopHeaveCoefficient;
            }
            controller.heave = Yibration() * HeaveCoefficient;
        }

        // 前回と今回のギア数が変わっている場合フラグをオンにする
        if (transmission.Gear != BeforeGear) ShiftChange = true;

        // ギア数が変わっているフラグがオンの場合
        if (ShiftChange)
        {
            // シフトショックの計算を行う
            float shiftValue = ShiftShock();

            // トランスミッションがオートの場合
            if (transmission.transmissionType == Transmission.TransmissionType.Automatic)
            {
                // オートマチック用の係数をかけてクランプする
                shiftValue = Mathf.Clamp(shiftValue * ShiftCoefficient_AT, -maxRotate, maxRotate);
            }
            // トランスミッションがマニュアルの場合
            else if (transmission.transmissionType == Transmission.TransmissionType.Manual)
            {
                // マニュアル用の係数をかけてクランプする
                shiftValue = Mathf.Clamp(shiftValue * ShiftCoefficient_MT, -maxRotate, maxRotate);
            }
            // 値を保存する
            ResultStorage = controller.pitch;
            // 値を引く
            controller.pitch -= shiftValue;
            // 前回シフトチェンジを行ったフラグをオンにする
            BeforeShiftChange = true;
        }
        // 前回シフトチェンジを行っている場合
        else if (BeforeShiftChange)
        {
            ShiftBackCounter += 1.0f;

            if (ShiftBackCounter > ShiftBackCount)
            {
                // 保存していた値を足す
                controller.pitch = ResultStorage;
                // 前回シフトチェンジを行ったフラグをオフにする
                BeforeShiftChange = false;
                ShiftBackCounter = 0.0f;
            }
        }

        // 速度を設定した通常速度にセットする
        controller.speedAxis123 = ChairNormalSpeed;

        //速度・加速度用に各要素の中で一番変化量が大きい値をを算出する
        float rollVlue = (targetPos.x - controller.roll);       //ロールの変化量
        float pitchVlue = (targetPos.y - controller.pitch);     //ピッチの変化量
        float yawVlue = (targetPos.z - controller.yaw);         //ヨーの変化量
        /*float HeaveVlue = (PreHeave - controller.heave);*/         //ヨーの変化量
        //一番大きい変化量を算出
        float MaxVule = Mathf.Abs(rollVlue) > Mathf.Abs(pitchVlue) ? Mathf.Abs(rollVlue) : Mathf.Abs(pitchVlue) > Mathf.Abs(yawVlue) ? Mathf.Abs(pitchVlue) : Mathf.Abs(yawVlue) /*> Mathf.Abs(HeaveVlue) ? Mathf.Abs(yawVlue) : Mathf.Abs(HeaveVlue)*/;

        // 特定の条件の時のみ、値を強制的に変更する
        float RollBet = Mathf.Abs(BeforeRoll - targetPos.x);
        float PicthBet = Mathf.Abs(BeforePicth - targetPos.y);
        //if (RollBet < UpdateRollThreshold && PicthBet < UpdatePicthThreshold)
        //{
        //    controller.speedAxis123 = DefoSpeed;
        //    controller.accelAxis123 = DefoAccel;
        //}

        if (CarStart)
        {
            if (pitchVlue > 0.3f)
            {
                //速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.speedAxis123 = DefoSpeed * Mathf.Clamp(Mathf.Abs(pitchVlue), MinSpeed / DefoSpeed, 1.0f);

                //加速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.accelAxis123 = DefoAccel * Mathf.Clamp(Mathf.Abs(pitchVlue), MinAccel / DefoAccel, 1.0f);
            }
            else if (Mathf.Abs(MaxVule) > 0.5f)
            {
                //速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.speedAxis123 = DefoSpeed * Mathf.Clamp(Mathf.Abs(0.5f), MinSpeed / DefoSpeed, 1.0f);

                //加速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.accelAxis123 = DefoAccel * Mathf.Clamp(Mathf.Abs(0.5f), MinAccel / DefoAccel, 1.0f);
            }
            else
            {
                //速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.speedAxis123 = DefoSpeed * Mathf.Clamp(Mathf.Abs(MaxVule), MinSpeed / DefoSpeed, 1.0f);

                //加速度を変化量の割合を算出しながらクランプ（Defo~Minまでにクランプ）
                controller.accelAxis123 = DefoAccel * Mathf.Clamp(Mathf.Abs(MaxVule), MinAccel / DefoAccel, 1.0f);
            }
        }
        else
        {
            controller.speedAxis123 = 0.5f;
            controller.accelAxis123 = 0.1f;
        }

        // 各種値が上下限値を超えないように補正する(ほぼ保険)
        controller.roll = Mathf.Clamp(controller.roll, -maxRotate, maxRotate);
        controller.sway = Mathf.Clamp(controller.sway, -maxRotate, maxRotate);
        controller.pitch = Mathf.Clamp(controller.pitch, -maxRotate, maxRotate);
        controller.surge = Mathf.Clamp(controller.surge, -maxRotate, maxRotate);
        controller.yaw = Mathf.Clamp(controller.yaw, -maxRotate, maxRotate);
        controller.heave = Mathf.Clamp(controller.heave, -maxRotate, maxRotate);
        controller.speedAxis123 = Mathf.Clamp(controller.speedAxis123, 0, 1);
        controller.accelAxis123 = Mathf.Clamp(controller.accelAxis123, 0, 1);
        controller.rotationMotionRatio = 1.0f;
        controller.gravityMotionRatio = 1.0f;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 速度が変わった後の処理
    // 引数１：重心のｚ軸値
    //━━━━━━━━━━━━━━━━━━━━━
    public void Accel(float _z)
    {
        //？アクセル中とブレーキ中の変数作成
        bool bAccel = false;
        bool bBrake = false;

        // ブレーキとアクセルの値を調整
        //？ブレーキの入力が 0.01 以上　又は　Ｓキー入力がある　又は　↓キー入力があるなら　（つまりブレーキ時）
        if (input.inputBrake > 0.01f || Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {

            _z *= brakeCoefficient; //？重心のZ軸値 = ブレーキの調整用係数(0.5f)
            bBrake = true;          //？ブレーキ中フラグをオンにする
        }
        //？アクセルの入力が 0.01f 以上　又は　Wキー入力がある　又は　↑キー入力があるなら　(つまりアクセル時)
        else if (input.inputAxel > 0.01f || Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            _z *= accelCoefficient; //？重心のZ軸値 = アクセルの調整用係数(0.5f)
            bAccel = true;          //？アクセル中フラグをオンにする
        }

        //？temp = クランプ(Z軸の重心値 * 前後の動き調整用係数 / 2.5g * 全体の移動量調整用係数, -1, 1);
        //？↑を要約すると、temp = -1 ～ 1 の間の値が代入される
        float temp = Mathf.Clamp(_z * farwordCoefficient / 2.5f * OverallTravelFactor, -maxRotate, maxRotate);

        //？Z軸移動をしたか
        if (bBalancePointMoved_z)
        {
            //？移動先の座標.Z が 0 より大きい　かつ　temp が 0 より大きい　かつ　temp　が移動先の座標.Z よりも小さい　かつ　アクセル中なら
            if (targetPos.z > 0 && temp > 0 && temp < targetPos.z && bAccel)
            {
                return;
            }
            //？移動先の座標.Z が 0 より小さい　かつ　temp が 0 より小さい かつ temp が 移動先の座標.Z よりも大きい かつ ブレーキ中なら
            else if (targetPos.z < 0 && temp < 0 && temp > targetPos.z && bBrake)
            {

                /*---------------------------------------------------
                 *記述者：池田
                 * この条件は不要では？
                 * 下記の条件式をコメント化しました。
                 
                //移動先の座標.Z が　0 よりも大きいなら
				if (targetPos.z > 0)
				{
					targetPos.z = -0.1f;
				}
				---------------------------------------------------*/
                return;
            }
        }

        // カウンターをリセット
        zAxisCnt = 0;

        // フラグを立てる
        bBalancePointMoved_z = true;

        // 移動した後の値を計算する
        targetPos.z = temp;

        // アクセルを踏んでいるかをチェック
        if (bAccel)
        {
            // アクセルを踏んでいるのに、椅子が前に傾けるのを防ぐ
            if (targetPos.z < 0.2f) targetPos.z = 0.2f;

            // 車が動き始める時の傾ける量を大きくする
            //？二つのベクトルの乗算の結果(前への速度の計算)が 5以下 なら
            if (Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward) < 5)
            {
                //？移動先の座標.Z = クランプ(移動先の座標.Z * 1.5f, -1, 1)つまり -1～1 の間の値を代入
                targetPos.z = Mathf.Clamp(targetPos.z * 1.5f, -maxRotate, maxRotate);
            }
        }
        //ブレーキ中なら
        else if (bBrake)
        {
            //？移動先の座標.Z が ０ 以上なら
            if (targetPos.z > 0)
            {
                targetPos.z = -0.1f;//？移動先の座標.Zを -0.1f で代入
            }
        }
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 車が曲がるときの処理
    // 引数１：重心のｘ軸値
    //━━━━━━━━━━━━━━━━━━━━━
    public void Steering(float _x)
    {
        // フラグを立てる
        bBalancePointMoved_x = true;

        // 移動した後の値を計算する
        //.x = Mathf.Clamp(_x / 2.5f * rightCoefficient * OverallTravelFactor, -maxRotate, maxRotate);
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 新 車が曲がるときの処理(Roll)
    //━━━━━━━━━━━━━━━━━━━━━
    public float LoadSteering()
    {
        if (targetPos.x != float.NaN)
        {
            // 前回の値を保存する
            BeforeRoll = targetPos.x;
        }

        bBalancePointMoved_x = true;    //左右の処理中に
        float temp;                     //Left・Rightの判断に一時的に使用
        float LeftLoad = load.forceCoef_Left;
        float RightLoad = load.forceCoef_Right;
        float between = Mathf.Abs(load.forceCoef_Right - load.forceCoef_Left);

        //左右の判定（荷重の大きい方を使用）
        if (RightLoad < LeftLoad)
        {
            // 値を拡張
            float load = Mathf.InverseLerp(0.5f, 1.0f, LeftLoad);

            // 今回と前回の和(絶対値)が更新する値より大きければ
            if (Mathf.Abs(load - BeforeRoll) > UpdateRollThreshold)
            {
                // 今回の荷重を適用(左に倒すため、マイナスの値を格納)
                temp = -load;
            }
            // そうでなければ
            else
            {
                // 前回の荷重を適用
                temp = BeforeRoll;
            }
        }
        else
        {
            // 値を拡張
            float load = Mathf.InverseLerp(0.5f, 1.0f, RightLoad);

            // 今回と前回の差(絶対値)が更新する値より大きければ
            if (Mathf.Abs(load - BeforeRoll) > UpdateRollThreshold)
            {
                // 今回の荷重を適用
                temp = load;
            }
            // そうでなければ
            else
            {
                // 前回の荷重をそのまま適用
                temp = BeforeRoll;
            }
        }

        // 荷重の差が小さいときは椅子のRollを0にする
        if (between < CarRollValue)
        {
            temp = 0.0f;
        }

        //移動した後の値を計算
        targetPos.x = Mathf.Clamp(temp, -maxRotate, maxRotate);
        Debug.Log($"%_targetPos.x : {targetPos.x}");
        return targetPos.x;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 車上下の振動
    // 引数１：重心のｙ軸値
    //━━━━━━━━━━━━━━━━━━━━━
    public void Vibration(float _y)
    {
        // フラグを立てる
        bBalancePointMoved_y = true;

        // 移動した後の値を計算する
        //targetPos.y = Mathf.Clamp(_y / 5 * upCoefficient * OverallTravelFactor, -maxRotate, maxRotate);
    }


    //━━━━━━━━━━━━━━━━━━━━━
    // 新 車が曲がるときの処理(前後)
    //━━━━━━━━━━━━━━━━━━━━━
    public float PitchCalc()
    {
        if (targetPos.y != float.NaN)
        {
            // 前回の値を保存
            BeforePicth = targetPos.y;
        }
        // Frontの荷重を取得
        float Front = load.forceCoef_Front;
        // Rearの荷重を取得
        float Rear = load.forceCoef_Rear;
        // 計算した値を保存するための変数
        float CalcValue = BeforePicth;
        // 補正をかけるための車の速度を取得
        float carSpeed = ForwardSpeed;
        // アクセルの値を取得
        float accel = VC.input.Accel;
        // ブレーキの値を取得
        float brake = VC.input.Brake;

        // 現在の車の速度が最低速度よりも遅かったら
        if (Mathf.Abs(carSpeed) < carSpeedMin)
        {
            // 値を0に補正する
            carSpeed = 0.0f;
        }

        // 加速の影響力が強い場合
        if (accel - brake > 0.0f)
        {
            if (CarStart)
            {
                // 0から10の範囲に補正する
                float scaleSpeed = Mathf.Clamp(carSpeed, 0.0f, 15.0f);
                // 0.1から1の範囲にスケーリングする
                scaleSpeed = ScaleValue(scaleSpeed, 0.0f, 15.0f, 0.1f, 1.0f);

                if (BeforePicth < MaxRearPicth)
                {
                    // Rearの荷重を適用する
                    CalcValue = Rear/* * scaleSpeed*/;

                    // 今回の値と前回の値の差が更新する値よりも小さかったら
                    if (Mathf.Abs(CalcValue - BeforePicth) < UpdatePicthThreshold)
                    {
                        // 前回の値を保存する
                        CalcValue = BeforePicth;
                    }

                }
                else
                {
                    // 前回の値を保存する
                    CalcValue = BeforePicth;
                }

                // 停止直後フラグのリセットがされていない場合
                if (!PitchFirstStop)
                {
                    // 次に車が止まった時、停止直後になるようにフラグをリセットする
                    PitchFirstStop = true;
                }

            }
            else
            {
                // 前回の値を保存する
                CalcValue = BeforePicth;
            }
        }

        // 減速の影響力が強い場合
        else if (accel - brake < 0.0f)
        {
            if (carSpeed > carSpeedMin)
            {
                // 0から10の範囲に補正する
                float scaleSpeed = Mathf.Clamp(carSpeed, 0.0f, 10.0f);
                // 0.1から1の範囲にスケーリングする
                scaleSpeed = ScaleValue(carSpeed, 0.0f, 10.0f, 0.0f, 1.0f);
                // スケーリングした速度とブレーキの値を足した後、0.1から1の範囲にスケーリングする
                float scaleValue = ScaleValue(scaleSpeed + brake, 0.0f, 2.0f, 0.1f, 0.5f);

                // 前方の荷重に計算した補正値をかけて現在の値から減算する
                CalcValue -= Front * scaleValue;

                // 現在の値と前回の値が更新値より小さい場合
                if (Mathf.Abs(CalcValue - BeforePicth) < UpdatePicthThreshold)
                {
                    // 前回の値に更新する
                    CalcValue = BeforePicth;
                }

                // 現在の値が前方への最大値を超えたら
                if (CalcValue < MaxFrontPicth)
                {
                    // 最大値に補正する
                    CalcValue = MaxFrontPicth;
                }
            }
            else if (PitchFirstStop)
            {
                CalcValue = -0.1f;
                PitchFirstStop = false;
            }
            else
            {
                CalcValue = 0.0f;
            }
        }

        // 影響力が同じ場合
        else
        {
            // 0から20の範囲に補正する
            float scaleSpeed = Mathf.Clamp(carSpeed, 0.0f, 20.0f);
            // 0.0から1の範囲にスケーリングする
            scaleSpeed = ScaleValue(carSpeed, 0.0f, 20.0f, 0.2f, 0.0f);
            // 前回の移動位置からスケーリングした値を引く
            float calc = CalcValue - scaleSpeed;
            Debug.Log($"b_calc : {calc}");
            // 計算結果が0より小さかったら0に補正する
            if (calc <= 0.0f) CalcValue = 0.0f;
            // そうでない場合
            else
            {
                // 計算結果と前回の値の差が更新する値より小さかったら
                if (Mathf.Abs(calc - BeforePicth) < UpdatePicthThreshold)
                {
                    // 前回の値を保存する
                    CalcValue = BeforePicth;
                }
                // そうでない場合
                else
                {
                    // 今回の計算結果を適用する
                    CalcValue = calc;
                }
            }

            // 現在の値が0.1f以下かつ車の速度が最低速度より大きければ
            if (CalcValue <= 0.2f && carSpeed > carSpeedMin)
            {
                // 車の速度が5より小さい場合
                if (carSpeed < 5.0f)
                {
                    // 値を0.1に補正する
                    CalcValue = 0.1f;
                }
                // そうでなければ
                else
                {
                    // 値を0.2に補正する
                    CalcValue = 0.2f;
                }

            }
            // 車が停止している場合
            else if (carSpeed < carSpeedMin)
            {
                // 値を0に補正する
                CalcValue = 0.0f;
            }
        }

        // -1から1の間に補正
        CalcValue = Mathf.Clamp(CalcValue, -1.0f, 1.0f);

        //移動した後の値を計算
        targetPos.y = Mathf.Clamp(CalcValue, -maxRotate, maxRotate);

        return targetPos.y;
    }

    public float PitchCalcTest()
    {
        float result = controller.pitch;

        // 車が動いた瞬間
        if (CarStart && CarStart != BeforePicthFlag)
        {
            // 少し後ろに倒す
            result += MovePitchValue;

            // 倒した値が最大Pitchを超えていたら
            if (result > MaxPitch)
            {
                // 最大量に補正する
                result = MaxPitch;
            }
        }

        // 前方への速度が最低速度より小さいかつ倒れている量が0を超えていたら
        if (ForwardSpeed < MinSpeed && result > 0.0f)
        {
            // 少し前に倒す(ブレーキの踏み込み量によって倒す量を変更する)
            result -= MovePitchValue * VC.input.Brake;
            if (result < 0.0f)
            {
                result = 0.0f;
            }
        }
        // 今回のフラグ内容を保存する
        BeforePicthFlag = CarStart;
        // 値を返す
        return result;

        //// 今回と前回の速度の差を計算する
        //float BetSpeed = (ForwardSpeed - BeforeSpeed) * 10.0f;

        //Debug.Log($"b_BetSpeed : {BetSpeed}");

        //// 現在の値を保存する
        //float resultValue = targetPos.y;

        //// 値の差が更新する範囲よりも大きい場合
        //if(BetSpeed > UpdatePicthThreshold)
        //{
        //    // 現在の値が後方最大Picthよりも小さい場合
        //    if(resultValue < MaxRearPicth)
        //    {
        //        // 値の差が5より小さい場合
        //        if(BetSpeed < 5.0f)
        //        {
        //            // 値を足す
        //            resultValue += AddPicth;
        //        }
        //        // 値の差が5より大きく10より小さい場合
        //        else if(BetSpeed < 10.0f)
        //        {
        //            // 値を二倍して足す
        //            resultValue += AddPicth * 2.0f;
        //        }
        //        // 値の差が10より大きい場合
        //        else
        //        {
        //            // 値を三倍して足す
        //            resultValue += AddPicth *= 3.0f;
        //        }
        //    }
        //}

        //// 最終結果を返す
        //return resultValue;
    }

    public float SurgeCalc()
    {
        float resultCalc = 0.0f;
        // 前回の値を保存
        BeforeSurge = controller.surge;

        // 加速度の算出
        carAcceleration = (ForwardSpeed - BeforeSpeed) / (Time.time - prevTime);
        /*
                if (Mathf.Abs(carAcceleration) < MinSpeed)
                {
                    carAcceleration = 0.0f;
                }*/

        Debug.Log($"carAcceleration : {carAcceleration}");

        if (ForwardSpeed > MinSpeed) resultCalc = Mathf.Clamp(Mathf.InverseLerp(MinSpeed, MaxSpeed, ForwardSpeed), 0.0f, 0.6f);
        if (carAcceleration < -MinfAccel)
        {
            resultCalc -= Mathf.InverseLerp(MinfAccel, MaxfAccel, Mathf.Abs(carAcceleration));
        }
        /*resultCalc = Mathf.Clamp(carAcceleration, -10.0f, 10.0f);
        resultCalc = ScaleValue(resultCalc, -10.0f, 10.0f, -0.2f, 0.5f);
*/
        float calc = Mathf.Abs(BeforeSurge - resultCalc);

        if ((calc < UpdateSurgeThreshold /*|| !CarStart*/) && resultCalc != 0.0f)
        {
            resultCalc = BeforeSurge;
        }

        // 今回の経過時間を保存
        prevTime = Time.time;

        // 今回の速度を保存
        BeforeSpeed = ForwardSpeed;

        Debug.Log($"resultCalc : {resultCalc}");

        /*resultCalc = Mathf.Clamp(resultCalc, -1.0f, 1.0f);*/

        return resultCalc;
    }

    // ---------------------------
    // Yawの回転
    // ---------------------------
    public float YawValue()
    {
        // アクセルの入力を取得する
        float accel = VC.input.Accel;
        float carSpeed = ForwardSpeed;

        // 現在の車の速度が最低速度よりも遅かったら
        if (Mathf.Abs(carSpeed) < carSpeedMin)
        {
            // 値を0に補正する
            carSpeed = 0.0f;
        }

        // 車が発進している場合
        if (CarStart)
        {
            // アクセルが踏まれていないかつ車が止まっている時は前回の値を返す
            if (accel < 0.01f && carSpeed < carSpeedMin) return BeforeYaw;
        }
        else
        {
            // 車が止まっている時
            if (carSpeed < carSpeedMin)
            {
                //前回の値を返す
                return BeforeYaw;
            }
        }

        // スリップ角を取得
        float value = VC.SlipAngle / ChangeCount_Yaw;
        VC.SlipAngle = 0.0f;

        if (carSpeed < YawChangeMinSpeed)
        {
            value = 0.0f;
        }
        //value *= Mathf.Clamp(Mathf.Abs(carSpeed),0.0f, 2.0f);

        // 指定した値に補間
        value = Mathf.Clamp(value, -SlipClamp, SlipClamp);

        // 0から1にスケーリングする
        value = ScaleValue(value, -SlipClamp, SlipClamp, -1.0f, 1.0f);

        // 念のため補間
        targetPos.z = Mathf.Clamp(value, -maxRotate, maxRotate);

        Debug.Log($"carSpeed : {carSpeed}");

        return targetPos.z;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // シフトをチェンジするときの処理
    //━━━━━━━━━━━━━━━━━━━━━
    public void ShiftUp()
    {
        if (Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward) < 5) return;

        bBalancePointMoved_z = true;

        // 前へ
        controller.pitch -= 0.3f * shiftUpCoefficient * speedCoefficient.z * OverallTravelFactor;
    }
    public void ShiftDown()
    {
        if (Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward) < 5) return;

        bBalancePointMoved_z = true;

        // 前への速度の計算
        float speed = Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward);

        float temp = (int)(Mathf.Abs(speed + 5) / 10);

        // 速度が低いほど前への力が強くなる
        controller.pitch -= 0.3f / temp * shiftDownCoefficient * speedCoefficient.z * OverallTravelFactor;
    }

    public float ShiftShock()
    {
        float resultBox = 0.0f;
        // 車の速度を取得
        float carSpeed = ForwardSpeed;

        // 車の速度が最低速度より小さかったら
        if (carSpeed < carSpeedMin)
        {
            // 速度を0にする
            carSpeed = 0.0f;
        }

        // 車の速度を補正する(0から設定された最大速度の範囲)
        carSpeed = Mathf.Clamp(carSpeed, 0.0f, ShiftMaxSpeed);

        // 補正した速度をスケーリングする
        carSpeed = ScaleValue(carSpeed, 0.0f, ShiftMaxSpeed, 0.0f, ShiftMaxScale);

        // ギアの値を0から1にスケーリングする
        float ScaleGear = ScaleValue(NowGear, 0.0f, 7.0f, 0.5f, 1.0f);

        // 車の速度とギアの値を足して0から1にスケーリングする
        float totalScale = ScaleValue(carSpeed + ScaleGear, 0.0f, 2.0f, 0.0f, 1.0f);

        // シフトアップの場合
        if (NowGear - BeforeGear > 0.0f)
        {
            // 値を計算する
            resultBox = ShittPitch * totalScale;
        }
        // シフトダウンの場合
        else
        {
            // 値を計算する(負の値)
            resultBox = -ShittPitch * totalScale;
        }

        // シフトチェンジのフラグをオフにする
        ShiftChange = false;

        // 値を返す
        return resultBox;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 壁に当たった後に呼び出されるの処理
    // 引数１：右壁に当たった→true
    // 　　　　左壁に当たった→false
    //━━━━━━━━━━━━━━━━━━━━━
    public void HitWall(bool _right)
    {
        /*-------------------------------------------------------
            壁に当たったらroll swayを+-する。30フレームくらいで戻す
        2023/09/06 池田
            -------------------------------------------------------*/
        // すでに処理している場合は後の処理をパスします
        if (bGotHit) return;

        // 壁の処理をするフラグがオフの場合、以下の処理を行わない
        if (!CallWall) return;

        float speed = Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward);    //車のスピード
        float temp = Mathf.Clamp((UnityEngine.Random.value - 0.5f) * (speed / 10), MinRange, MaxRange);    //椅子の移動量 -0.5~0.5 でクランプ

        controller.speedAxis123 = 1f;
        controller.accelAxis123 = 1f;

        //当たったのは右壁か左壁か　true:右　false:左
        if (_right)
        {
            controller.roll -= temp;
            controller.sway += temp;
            saveHitMove = temp;
            bLR = true;

            //controller.roll -= saveHitMove;
            //controller.sway += saveHitMove;
        }
        else
        {
            controller.roll += temp;
            controller.sway -= temp;
            saveHitMove = temp;
            bLR = false;

            //controller.roll += saveHitMove;
            //controller.sway -= saveHitMove;
        }

        bGotHit = true;//処理済み


        ////前への速度の計算
        //float speed = Vector3.Dot(rootRigidbody.velocity, transform1.transform.forward);
        ////計算に使う、左右どちらの壁に当たったかを判別する為の変数
        //Vector2 temp = new Vector2(-1, 1);
        ////右の壁に当たったなら
        //if (_right)
        //{
        //    temp.x = -temp.x;
        //}
        ////衝突時のスピードにより椅子の動きの速さを変更(08/28 池田)
        //if (speed < 20.0f)
        //{
        //    controller.accelAxis123 = 0.5f; //加速度
        //    controller.speedAxis123 = 0.5f;
        //}
        //else if (speed < 25.0f)
        //{
        //    controller.accelAxis123 = 0.7f; //加速度
        //    controller.speedAxis123 = 0.7f;
        //}
        //else if (speed < 30.0f)
        //{
        //    controller.accelAxis123 = 0.9f; //加速度
        //    controller.speedAxis123 = 0.9f;
        //}
        //else
        //{
        //    controller.accelAxis123 = 1.0f; //加速度
        //    controller.speedAxis123 = 1.0f;
        //}
        ////壁にぶつかった時のスピードに応じて揺れ具合を変更する
        ////ぶつかった壁の方向に椅子を動かす
        //// 当たった後各軸の値を計算し、一定の範囲の中にした後保存しておく
        ////-1～1 の間でクランプ、(各椅子の回転 - temp * 前への速度 / 60(１フレーム？) * ぶつかった後の衝撃調整用係数)
        //pitchAfterHit = Mathf.Clamp(controller.pitch - temp.y * speed / 60 * impactCoefficient.y, -maxRotate, maxRotate);
        //rollAfterHit = Mathf.Clamp(controller.roll - temp.x * speed / 60 * impactCoefficient.x, -maxRotate, maxRotate);
        //yawAfterHit = Mathf.Clamp(controller.yaw + temp.x * speed / 60 * impactCoefficient.x, -maxRotate, maxRotate);

        //// 計算結果を各軸に代入
        //controller.pitch = pitchAfterHit;
        //controller.roll = rollAfterHit;
        //controller.yaw = yawAfterHit;

        ////処理済みフラグを立てる
        //bGotHit = true;
        ////カウンターをリセット
        //hitCnt = 0;



    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 車の振動
    // 引数１：今の車の速度
    //━━━━━━━━━━━━━━━━━━━━━
    private void Rolling(float _nowSpeed)
    {
        //// 速度で振動の間隔を取得し、保存しておく
        //float RollingCntThreshold = (int)rollingCurve.Evaluate(_nowSpeed);

        //// もし間隔が、30以上になっていたら、後の処理はしない
        //if (RollingCntThreshold >= 30) return;

        //// カウンターで間隔を計算
        //if (++RollingCnt < RollingCntThreshold) return;

        //// カウンターのリセット
        //RollingCnt = 0;




        FreamCounter++; //フレーム毎にカウント

        //カウントが２０以上なら
        if (FreamCounter > 20.0f)
        {
            // 上下
            //？椅子の上下移動 += クランプ((0～１のランダムな値 - 0.5f) * 2 / 0.05 * 10 / 今のスピード * 振動の激しさ調整用係数.Y * 全体の移動量を調整用の係数 , -0.05 , 0.05)
            //？↑は要するに　椅子の上下移動量に　-0.05 ～ 0.05 を足すということ
            float temp = Mathf.Clamp((UnityEngine.Random.value - 0.5f) * 2 / RollingRandomRange * 10 / _nowSpeed * RollingCoefficient.y * OverallTravelFactor, ((RollingRandomRange / 2) + (RollingRandomRange / 4)), RollingRandomRange);
            controller.heave += temp;
            controller.heave -= temp;
            FreamCounter = 0;
        }




        // 前後
        //？椅子の前後回転 += クランプ((ランダムな値 - 0.5f) * 2 / 0.05 * 10 / 今のスピード * 振動の激しさ調整用係数.Z * 全体の移動量を調整用の係数(0.5f) , -0.05 , 0.05)
        //？↑は要するに　椅子の前後回転に　-0.05 ～ 0.05　を足すということ
        //controller.pitch += Mathf.Clamp((UnityEngine.Random.value - 0.5f) * 2 / RollingRandomRange * 10 / _nowSpeed * RollingCoefficient.z * OverallTravelFactor, -RollingRandomRange, RollingRandomRange);

        // 上下
        //？椅子の上下移動 += クランプ((ランダムな値 - 0.5f) * 2 / 0.05 * 10 / 今のスピード * 振動の激しさ調整用係数.Y * 全体の移動量を調整用の係数 , -0.05 , 0.05)
        //？↑は要するに　椅子の上下移動量に　-0.05 ～ 0.05 を足すということ　
        //controller.heave += Mathf.Clamp((UnityEngine.Random.value - 0.5f) * 2 / RollingRandomRange * 10 / _nowSpeed * RollingCoefficient.y * OverallTravelFactor, -RollingRandomRange, RollingRandomRange);

        // 左右
        //？椅子の左右回転 temp = -0.05 ～ 0.05 でクランプした値　計算は上記のものと同じ　
        //float temp = Mathf.Clamp((UnityEngine.Random.value - 0.5f) * 2 / RollingRandomRange * 10 / _nowSpeed * RollingCoefficient.x * OverallTravelFactor, -RollingRandomRange, RollingRandomRange);
        //controller.roll += temp;    //？椅子の左右回転 += 上記の式で求めた temp　
        //controller.yaw -= temp;     //？椅子の上下回転 -= 上記の式で求めた temp　
    }

    // ----------------------------------
    // sin波を使用した上下の振動量
    // ----------------------------------
    private float Yibration()
    {
        // 補正をかけるために車の速度を取得する
        float carSpeed = ForwardSpeed;
        // 0から20の範囲に補正する
        carSpeed = Mathf.Clamp(carSpeed, 0.0f, 20.0f);
        // 0.02から0.1の範囲にスケーリングする
        carSpeed = ScaleValue(carSpeed, 0.0f, 20.0f, 0.02f, 0.1f);

        // 振幅数を計算する
        float f = 1.0f / Frequency;

        // sin波を作成する
        float sin = carSpeed * Mathf.Sin(2 * Mathf.PI * f * Time.fixedTime);

        //sin = Mathf.Clamp(carSpeed, -0.5f, 0.5f);

        // 値が小さすぎる場合
        if (sin > -0.001f && sin < 0.001f)
        {
            if (sin > 0.0f)
            {
                // 0に補正する
                sin = 0.001f;
            }
            else
            {
                // 0に補正する
                sin = -0.001f;
            }
        }

        // -1から1の間に補正する
        //sin = Mathf.Clamp(sin, -1.0f, 1.0f);

        // 値を返す
        return sin;
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 地面の減速帯による車(椅子)の振動
    //引数1：現在のスピード
    //引数2：揺れ方(string  UpDown：交互　Long：継続(しばらく赤い減速帯))
    //━━━━━━━━━━━━━━━━━━━━━
    /*----------------------------------------------------------------
	作成日：2023/08/22
	担当：21cu0203_池田柊太
	概要：地面の減速帯による椅子の揺れを作成するための関数追加
	 ----------------------------------------------------------------*/

    public void DecelerationRolling(float _nowSpeed, string _type)
    {
        //0.02～0.05　の間から値を返す
        float temp = Mathf.Clamp((UnityEngine.Random.value - 0.5f) * 2 / RollingRandomRange * 10 / _nowSpeed * RollingCoefficient.y * OverallTravelFactor, ((RollingRandomRange / 2) + (RollingRandomRange / 4)), RollingRandomRange);
        Debug.Log(temp);

        //交互に切り替わる減速帯
        if (_type == "UpDown")
        {
            //アクセルを踏んでいる　//かつ　減速帯(交互)を踏んでいたら
            if (input.inputAxel > 0.01f)
            {
                FreamCounter++; //フレーム毎にカウント

                //カウントが10なら
                if (FreamCounter == 10.0f)
                {
                    controller.heave += temp;   //椅子を上昇
                }
                //カウントが20なら
                if (FreamCounter == 20.0f)
                {
                    controller.heave -= temp;   //椅子を下降
                    FreamCounter = 0;           //カウントリセット
                }
            }
        }
        //赤い減速帯が継続
        else if (_type == "Long")
        {
            //アクセルを踏んでいる　//かつ　減速帯(継続)を踏んでいたら
            if (input.inputAxel > 0.01f)
            {
                //一度だけ振動させる
                controller.heave += temp;
                controller.heave -= temp;
                //bUsed = true;
            }
        }


    }


    // ----------------------------------------------
    // ゴールした後の処理
    // ----------------------------------------------
    public void GoalRhairReset()
    {
        if (controller.pitch == 0.0f && controller.surge == 0.0f)
        {
            controller.speedAxis123 = GoalSpeed;
            controller.accelAxis123 = GoalAccel;

            controller.roll = 0.0f;
            controller.sway = 0.0f;
            controller.yaw = 0.0f;
            controller.heave = 0.0f;
            GoalEnd = true;
        }
        else
        {
            controller.speedAxis123 = GoalSpeed_ps;
            controller.accelAxis123 = GoalAccel_ps;

            controller.pitch = 0.0f;
            controller.surge = 0.0f;
        }

    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 減衰する Sin波
    // 引数１：時間
    // 引数２：波長
    // 引数３：減衰の関数
    // 戻り値：時間をグラフに代入した後の解
    //━━━━━━━━━━━━━━━━━━━━━
    private float DampedSinWave(float _time, float _wavelength, Func<float, float> _func)
    {
        return Mathf.Sin(_time / _wavelength * 2 * Mathf.PI) * _func(_time);
    }
    //━━━━━━━━━━━━━━━━━━━━━
    // 減衰する Sin波
    // 引数１：時間
    // 引数２：波長
    // 引数３：減衰のグラフ
    // 戻り値：時間をグラフに代入した後の解
    //━━━━━━━━━━━━━━━━━━━━━
    private float DampedSinWave(float _time, float _wavelength, AnimationCurve _curve)
    {
        return Mathf.Sin(_time / _wavelength * 2 * Mathf.PI) * _curve.Evaluate(_time);
    }

    //━━━━━━━━━━━━━━━━━━━━━
    // 減衰する Cos波
    // 引数１：時間
    // 引数２：波長
    // 引数３：減衰の関数
    // 戻り値：時間をグラフに代入した後の解
    //━━━━━━━━━━━━━━━━━━━━━
    private float DampedCosWave(float _time, float _wavelength, Func<float, float> _func)
    {
        return Mathf.Cos(_time / _wavelength * 2 * Mathf.PI) * _func(_time);
    }
    //━━━━━━━━━━━━━━━━━━━━━
    // 減衰する Cos波
    // 引数１：時間
    // 引数２：波長
    // 引数３：減衰のグラフ
    // 戻り値：時間をグラフに代入した後の解
    //━━━━━━━━━━━━━━━━━━━━━
    private float DampedCosWave(float _time, float _wavelength, AnimationCurve _curve)
    {
        return Mathf.Cos(_time / _wavelength * 2 * Mathf.PI) * _curve.Evaluate(_time);
    }

    // -----------------------------------------
    // 値を指定した範囲にスケーリングする関数
    // 引数1 : スケーリングを行う値
    // 引数2 : 現在の最小値
    // 引数3 : 現在の最大値
    // 引数4 : スケーリングする最小値
    // 引数5 : スケーリングする最大値
    // -----------------------------------------
    public float ScaleValue(float value, float NowMin, float NowMax, float ScaleMin, float ScaleMax)
    {
        // 値を新しい範囲に変換
        return ScaleMin + (value - NowMin) * (ScaleMax - ScaleMin) / (NowMax - NowMin);
    }

    public float Rand()
    {
        // 値を新しい範囲に変換
        return UnityEngine.Random.Range(-0.5f, 0.5f);
    }
}