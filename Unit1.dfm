object Form1: TForm1
  Left = 192
  Top = 107
  Caption = 'ESPSDR S3-Zero'
  ClientHeight = 662
  ClientWidth = 1004
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  Position = poDesktopCenter
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  OnShow = FormShow
  DesignSize = (
    1004
    662)
  PixelsPerInch = 96
  TextHeight = 13
  object Gauge1: TGauge
    Left = 280
    Top = 38
    Width = 150
    Height = 16
    Progress = 0
    ShowText = False
  end
  object pbScope: TPaintBox
    Left = 8
    Top = 64
    Width = 988
    Height = 520
    Anchors = [akLeft, akTop, akRight, akBottom]
    OnPaint = pbScopePaint
  end
  object edtDeviceCOM: TEdit
    Left = 8
    Top = 8
    Width = 70
    Height = 21
    TabOrder = 0
    Text = 'COM208'
  end
  object btnComportConnectDisconnect: TButton
    Left = 84
    Top = 6
    Width = 90
    Height = 25
    Caption = 'Connect'
    TabOrder = 1
    OnClick = btnComportConnectDisconnectClick
  end
  object edtCenterFreq: TEdit
    Left = 190
    Top = 8
    Width = 60
    Height = 21
    TabOrder = 2
    Text = '2440'
  end
  object dwnFreq: TButton
    Left = 254
    Top = 6
    Width = 32
    Height = 25
    Caption = '-'
    TabOrder = 3
    OnClick = dwnFreqClick
  end
  object upFreq: TButton
    Left = 290
    Top = 6
    Width = 32
    Height = 25
    Caption = '+'
    TabOrder = 4
    OnClick = upFreqClick
  end
  object cbSampleRate: TComboBox
    Left = 330
    Top = 8
    Width = 60
    Height = 21
    Style = csDropDownList
    ItemHeight = 13
    TabOrder = 5
    OnChange = cbSampleRateChange
    Items.Strings = (
      '80'
      '40'
      '16')
  end
  object cbFFT: TComboBox
    Left = 398
    Top = 8
    Width = 70
    Height = 21
    Style = csDropDownList
    ItemHeight = 13
    TabOrder = 6
    Items.Strings = (
      '512'
      '1024'
      '2048'
      '4096')
  end
  object edtFPS: TEdit
    Left = 474
    Top = 8
    Width = 40
    Height = 21
    TabOrder = 7
    Text = '10'
  end
  object btnStream: TButton
    Left = 522
    Top = 6
    Width = 75
    Height = 25
    Caption = 'Stream'
    TabOrder = 8
    OnClick = btnStreamClick
  end
  object btnStop: TButton
    Left = 603
    Top = 6
    Width = 60
    Height = 25
    Caption = 'Stop'
    TabOrder = 9
    OnClick = btnStopClick
  end
  object chkAGC: TCheckBox
    Left = 675
    Top = 10
    Width = 50
    Height = 17
    Caption = 'AGC'
    Checked = True
    State = cbChecked
    TabOrder = 10
    OnClick = chkAGCClick
  end
  object tbGain: TTrackBar
    Left = 730
    Top = 4
    Width = 150
    Height = 30
    Max = 82
    Frequency = 10
    Position = 40
    TabOrder = 11
    OnChange = tbGainChange
  end
  object ChkWateFall: TCheckBox
    Left = 890
    Top = 10
    Width = 80
    Height = 17
    Caption = 'Waterfall'
    Checked = True
    State = cbChecked
    TabOrder = 12
    OnClick = ChkWateFallClick
  end
  object StarScanEdit: TEdit
    Left = 8
    Top = 36
    Width = 60
    Height = 21
    TabOrder = 13
    Text = '1842'
  end
  object EndScanEdit: TEdit
    Left = 72
    Top = 36
    Width = 60
    Height = 21
    TabOrder = 14
    Text = '2790'
  end
  object edtStep: TEdit
    Left = 136
    Top = 36
    Width = 40
    Height = 21
    TabOrder = 15
    Text = '60'
  end
  object btnFastSweep: TButton
    Left = 182
    Top = 34
    Width = 90
    Height = 25
    Caption = 'Fast Sweep'
    TabOrder = 16
    OnClick = btnFastSweepClick
  end
  object Memo1: TMemo
    Left = 8
    Top = 590
    Width = 988
    Height = 70
    Anchors = [akLeft, akRight, akBottom]
    Color = clBlack
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clLime
    Font.Height = -11
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
    ScrollBars = ssVertical
    TabOrder = 17
  end
  object cbWidth: TComboBox
    Left = 436
    Top = 35
    Width = 80
    Height = 21
    Style = csDropDownList
    ItemHeight = 13
    TabOrder = 18
    OnChange = cbWidthChange
  end
  object btnChkRange: TButton
    Left = 522
    Top = 33
    Width = 75
    Height = 25
    Caption = 'ChkRange'
    TabOrder = 19
    OnClick = btnChkRangeClick
  end
  object btnIQStream: TButton
    Left = 603
    Top = 33
    Width = 60
    Height = 25
    Caption = 'IQStream'
    TabOrder = 20
    OnClick = btnIQStreamClick
  end
  object cbIQDecim: TComboBox
    Left = 666
    Top = 37
    Width = 65
    Height = 21
    Style = csDropDownList
    ItemHeight = 13
    TabOrder = 21
  end
  object chkPeakHold: TCheckBox
    Left = 737
    Top = 40
    Width = 97
    Height = 17
    Caption = 'PeakHold'
    TabOrder = 22
    OnClick = chkPeakHoldClick
  end
end
