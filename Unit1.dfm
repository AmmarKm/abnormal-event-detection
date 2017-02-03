object Form1: TForm1
  Left = 0
  Top = 0
  Width = 1247
  Height = 764
  AutoScroll = True
  Caption = 'Form1'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  Menu = MainMenu1
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object Label4: TLabel
    Left = 288
    Top = 350
    Width = 95
    Height = 19
    Caption = #33258#23450'OF'#38291#38548': '
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object Label16: TLabel
    Left = 936
    Top = 392
    Width = 104
    Height = 13
    Caption = 'abonomal label ignore'
  end
  object Label17: TLabel
    Left = 256
    Top = 294
    Width = 125
    Height = 19
    Caption = #31890#23376#22823#23567'(pscale):'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object Label18: TLabel
    Left = 299
    Top = 323
    Width = 79
    Height = 19
    Caption = 'motion_th:'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object Panel3: TPanel
    Left = 327
    Top = 514
    Width = 256
    Height = 108
    TabOrder = 33
    object Edit11: TEdit
      Left = 151
      Top = 76
      Width = 89
      Height = 21
      Enabled = False
      TabOrder = 0
      Text = '100'
    end
    object RadioGroup1: TRadioGroup
      Left = 0
      Top = 13
      Width = 145
      Height = 89
      Caption = 'RadioGroup1'
      ItemIndex = 1
      Items.Strings = (
        'socal force data'
        'bag of feature')
      TabOrder = 1
    end
  end
  object CheckBox1: TCheckBox
    Left = 288
    Top = 411
    Width = 146
    Height = 33
    Caption = 'non-zero motion only'
    TabOrder = 0
  end
  object CheckBox2: TCheckBox
    Left = 288
    Top = 387
    Width = 114
    Height = 25
    Caption = 'my feature points'
    Checked = True
    State = cbChecked
    TabOrder = 1
  end
  object Button8: TButton
    Left = 430
    Top = 391
    Width = 153
    Height = 73
    Caption = 'my OpticalFlow'
    TabOrder = 2
    OnClick = Button8Click
  end
  object Edit1: TEdit
    Left = 383
    Top = 350
    Width = 113
    Height = 21
    TabOrder = 3
    Text = '4'
  end
  object CheckBox3: TCheckBox
    Left = 502
    Top = 360
    Width = 81
    Height = 25
    Caption = 'cvWaitKey'
    TabOrder = 4
  end
  object Panel1: TPanel
    Left = 609
    Top = 287
    Width = 169
    Height = 163
    TabOrder = 5
    object Label5: TLabel
      Left = 26
      Top = 6
      Width = 45
      Height = 19
      Caption = 'width:'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label6: TLabel
      Left = 21
      Top = 31
      Width = 50
      Height = 19
      Caption = 'height:'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label7: TLabel
      Left = 44
      Top = 56
      Width = 27
      Height = 19
      Caption = 'fps:'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label8: TLabel
      Left = 19
      Top = 81
      Width = 54
      Height = 19
      Caption = 'frames:'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label9: TLabel
      Left = 88
      Top = 6
      Width = 46
      Height = 19
      Caption = 'Label9'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label10: TLabel
      Left = 88
      Top = 31
      Width = 55
      Height = 19
      Caption = 'Label10'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label11: TLabel
      Left = 88
      Top = 56
      Width = 55
      Height = 19
      Caption = 'Label11'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label12: TLabel
      Left = 88
      Top = 81
      Width = 55
      Height = 19
      Caption = 'Label12'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label13: TLabel
      Left = 16
      Top = 106
      Width = 56
      Height = 19
      Caption = 'current:'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label14: TLabel
      Left = 88
      Top = 106
      Width = 55
      Height = 19
      Caption = 'Label14'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
    object Label15: TLabel
      Left = 88
      Top = 128
      Width = 55
      Height = 19
      Caption = 'Label15'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -16
      Font.Name = 'Tahoma'
      Font.Style = []
      ParentFont = False
    end
  end
  object Memo1: TMemo
    Left = 581
    Top = 120
    Width = 313
    Height = 161
    ScrollBars = ssBoth
    TabOrder = 6
  end
  object Edit2: TEdit
    Left = 384
    Top = 323
    Width = 81
    Height = 21
    TabOrder = 7
    Text = '1.5'
  end
  object Edit3: TEdit
    Left = 384
    Top = 296
    Width = 81
    Height = 21
    TabOrder = 8
    Text = '3'
  end
  object Memo2: TMemo
    Left = 273
    Top = 8
    Width = 240
    Height = 261
    ScrollBars = ssBoth
    TabOrder = 9
  end
  object FileListBox1: TFileListBox
    Left = 24
    Top = 373
    Width = 226
    Height = 108
    FileType = [ftDirectory, ftNormal]
    ItemHeight = 13
    Mask = '*.wmv;*.avi;*.mpg'
    TabOrder = 10
    OnDblClick = FileListBox1DblClick
  end
  object Button16: TButton
    Left = 24
    Top = 487
    Width = 75
    Height = 25
    Caption = #36984#25799#36039#26009#22846
    TabOrder = 11
    OnClick = Button16Click
  end
  object Button19: TButton
    Left = 105
    Top = 487
    Width = 48
    Height = 25
    Caption = 'refresh'
    TabOrder = 12
    OnClick = Button19Click
  end
  object Button21: TButton
    Left = 801
    Top = 312
    Width = 106
    Height = 52
    Caption = 'testing'
    Enabled = False
    TabOrder = 13
    OnClick = Button21Click
  end
  object Memo3: TMemo
    Left = 1088
    Top = 241
    Width = 105
    Height = 129
    ScrollBars = ssBoth
    TabOrder = 14
  end
  object Memo4: TMemo
    Left = 928
    Top = 18
    Width = 121
    Height = 199
    Lines.Strings = (
      'ap')
    ScrollBars = ssBoth
    TabOrder = 15
  end
  object Memo5: TMemo
    Left = 1080
    Top = 16
    Width = 113
    Height = 201
    Lines.Strings = (
      'Z_average')
    ScrollBars = ssBoth
    TabOrder = 16
  end
  object Button22: TButton
    Left = 803
    Top = 415
    Width = 105
    Height = 49
    Caption = 'Refine'
    Enabled = False
    TabOrder = 17
    OnClick = Button22Click
  end
  object Memo6: TMemo
    Left = 936
    Top = 241
    Width = 146
    Height = 129
    Lines.Strings = (
      'Memo6')
    ScrollBars = ssBoth
    TabOrder = 18
  end
  object Edit5: TEdit
    Left = 803
    Top = 370
    Width = 104
    Height = 21
    TabOrder = 19
    Text = '1.5'
  end
  object Edit6: TEdit
    Left = 1056
    Top = 389
    Width = 41
    Height = 21
    TabOrder = 20
    Text = '13'
  end
  object Button15: TButton
    Left = 24
    Top = 518
    Width = 75
    Height = 25
    Caption = 'test2 Dir'
    TabOrder = 21
    OnClick = Button15Click
  end
  object Button23: TButton
    Left = 24
    Top = 549
    Width = 75
    Height = 25
    Caption = 'test4 Dir'
    TabOrder = 22
    OnClick = Button15Click
  end
  object Edit7: TEdit
    Left = 803
    Top = 285
    Width = 89
    Height = 21
    TabOrder = 23
    Text = 'output_bin_1'
  end
  object Edit9: TEdit
    Left = 288
    Top = 456
    Width = 121
    Height = 21
    TabOrder = 24
    Text = '0'
  end
  object Button7: TButton
    Left = 24
    Top = 580
    Width = 75
    Height = 25
    Caption = 'test6 half'
    TabOrder = 25
    OnClick = Button7Click
  end
  object Button25: TButton
    Left = 526
    Top = 171
    Width = 49
    Height = 33
    Caption = 'clear'
    TabOrder = 26
    OnClick = Button25Click
  end
  object CheckBox6: TCheckBox
    Left = 177
    Top = 487
    Width = 73
    Height = 25
    Caption = 'ImageSeq.'
    DoubleBuffered = False
    ParentDoubleBuffered = False
    TabOrder = 27
    OnClick = CheckBox6Click
  end
  object DirectoryListBox1: TDirectoryListBox
    Left = 24
    Top = 241
    Width = 226
    Height = 126
    FileList = FileListBox1
    TabOrder = 28
  end
  object Button26: TButton
    Left = 105
    Top = 580
    Width = 57
    Height = 25
    Caption = 'UCSD1'
    TabOrder = 29
    OnClick = Button26Click
  end
  object Panel2: TPanel
    Left = 612
    Top = 506
    Width = 309
    Height = 171
    TabOrder = 30
    object CheckBox4: TCheckBox
      Left = 17
      Top = 48
      Width = 105
      Height = 25
      Caption = #21435#38500#36942#23567#30340#20998#32676
      TabOrder = 0
    end
    object Edit10: TEdit
      Left = 40
      Top = 79
      Width = 73
      Height = 21
      TabOrder = 1
      Text = '5'
    end
    object Button18: TButton
      Left = 184
      Top = 9
      Width = 106
      Height = 59
      Caption = 'training'
      TabOrder = 2
      OnClick = Button18Click
    end
    object CheckBox7: TCheckBox
      Left = 17
      Top = 106
      Width = 136
      Height = 47
      Caption = 'Whitening (train && Test)'
      Checked = True
      State = cbChecked
      TabOrder = 3
      OnClick = CheckBox7Click
    end
    object CheckBox5: TCheckBox
      Left = 17
      Top = 9
      Width = 73
      Height = 25
      Caption = #25163#21205#36664#20837'K:'
      TabOrder = 4
      OnClick = CheckBox5Click
    end
    object Edit4: TEdit
      Left = 105
      Top = 13
      Width = 49
      Height = 21
      Enabled = False
      TabOrder = 5
      Text = '50'
    end
  end
  object Memo7: TMemo
    Left = 552
    Top = 16
    Width = 305
    Height = 89
    Lines.Strings = (
      'Memo7')
    ScrollBars = ssBoth
    TabOrder = 31
  end
  object Chart1: TChart
    Left = 16
    Top = 8
    Width = 249
    Height = 217
    Title.Text.Strings = (
      'BoF_DES (non-normalize)')
    View3D = False
    TabOrder = 32
    object Series1: TBarSeries
      BarPen.Visible = False
      Marks.Arrow.Visible = True
      Marks.Callout.Brush.Color = clBlack
      Marks.Callout.Arrow.Visible = True
      Marks.Visible = False
      ShowInLegend = False
      Gradient.Direction = gdTopBottom
      XValues.Name = 'X'
      XValues.Order = loAscending
      YValues.Name = 'Bar'
      YValues.Order = loNone
    end
  end
  object Edit8: TEdit
    Left = 439
    Top = 487
    Width = 105
    Height = 21
    TabOrder = 34
    Text = 'training_bin_1'
  end
  object Panel4: TPanel
    Left = 32
    Top = 622
    Width = 337
    Height = 76
    TabOrder = 35
    object Button1: TButton
      Left = 0
      Top = 24
      Width = 83
      Height = 41
      Caption = 'Button1'
      TabOrder = 0
      OnClick = Button1Click
    end
    object Button2: TButton
      Left = 105
      Top = 8
      Width = 113
      Height = 25
      Caption = 'Button2'
      TabOrder = 1
      OnClick = Button2Click
    end
    object Button3: TButton
      Left = 105
      Top = 39
      Width = 113
      Height = 33
      Caption = 'Button3'
      TabOrder = 2
      OnClick = Button3Click
    end
    object Button4: TButton
      Left = 224
      Top = 24
      Width = 73
      Height = 33
      Caption = 'Button4'
      TabOrder = 3
      OnClick = Button4Click
    end
  end
  object OpenPictureDialog1: TOpenPictureDialog
    Left = 1024
    Top = 504
  end
  object MainMenu1: TMainMenu
    Left = 952
    Top = 496
    object Menu1: TMenuItem
      Caption = 'Menu'
      object Exit1: TMenuItem
        Caption = 'Exit'
        OnClick = Exit1Click
      end
    end
  end
  object OpenDialog1: TOpenDialog
    Left = 1016
    Top = 440
  end
  object SaveDialog1: TSaveDialog
    Filter = 'all|*.*|txt|*.txt'
    FilterIndex = 2
    Left = 952
    Top = 432
  end
  object OpenDialog2: TOpenDialog
    Filter = 'visual word file (*.vw)|*.vw|all|*.*'
    Left = 1080
    Top = 440
  end
end
