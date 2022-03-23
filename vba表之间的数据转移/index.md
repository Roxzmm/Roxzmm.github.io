# VBA表之间的数据转移


Tags: #VBA #Excel  
Links:

---

# VBA表之间的数据转移

``` excel
Sub transfer()
    '使用前需要确保数据导出的工作本里都是需要的工作簿
	'如果有多余不需要的工作簿，则选中所有需要的工作簿，然后另存为新的工作本以供使用
	'需要确保两个工作本里表头的单元格首尾没有空格
	'如果有哪个表的店铺一栏是空的，得填一些内容上去，确保该列不为空
    
    Application.ScreenUpdating = False
    '选择单一文件
    With Application.FileDialog(msoFileDialogFilePicker)
        .AllowMultiSelect = False   '单选择
        .Filters.Clear   '清除文件过滤器
        .Filters.Add "Excel Files", "*.xlsx;*.xlsm;*.xls;*.xlw"
        .Filters.Add "All Files", "*.*"          '设置两个文件过滤器
        If .Show = -1 Then    'FileDialog 对象的 Show方法显示对话框，并且返回-1（如果你按OK）和0（如果你按Cancel）
'            MsgBox "您选择的文件是" & .SelectedItems(1), vbOKOnly + vbInformation, "智能Excel"
            sourcebookpath = .SelectedItems(1)
        End If
    End With
    
    Dim source As Workbook
    Dim sh1, sh2 As Worksheet
    Set sourcebook = Workbooks.Open(sourcebookpath)
    Set sh1 = ThisWorkbook.Sheets(1)
    Set sh2 = sourcebook.Sheets(1)
    columnMax1 = sh1.Range("IV1").End(xlToLeft).Column
    
    'sheets预处理（表格不规范时每次都要调整）-----------------------------------
    Dim usefulSheets1()
    ReDim usefulSheets1(1 To sourcebook.Worksheets.count)
    Dim countSheets1 As Integer
    countSheets1 = 1
    For i = 1 To sourcebook.Worksheets.count
        If (InStr(sourcebook.Sheets(i).Name, "ÍË¿î") = 0) And (InStr(sourcebook.Sheets(i).Name, "·Ñ") = 0) And (sourcebook.Sheets(i).Visible = True) Then
            Set usefulSheets1(countSheets1) = sourcebook.Sheets(i)
            countSheets1 = countSheets1 + 1
        End If
    Next
    Dim usefulSheets()
    ReDim usefulSheets(1 To countSheets1)
    Dim countSheets2 As Integer
    countSheets2 = 1
    For i = 9 To (countSheets1 - 1)
        Set usefulSheets(countSheets2) = usefulSheets1(i)
        countSheets2 = countSheets2 + 1
    Next
    '-----------------------------------------------------------

    '从表一转移数据到表二
    Dim existingRows As Integer
    existingRows = 3
    For i = 1 To (countSheets2 - 1)
        'Set sh2 = sourcebook.Sheets(i)
        Set sh2 = usefulSheets(i)
        
        '确定两张表的表头各自对应
        columnMax2 = sh2.Range("IV1").End(xlToLeft).Column
        Dim columnIndex()
        ReDim columnIndex(1 To columnMax1)
        '表头前两列-组别（平台）和店铺手动设置
        columnIndex(1) = 1
        columnIndex(2) = 2
        For j = 3 To columnMax1
            For k = 3 To columnMax2
                If Trim(sh1.Cells(2, j)) = Trim(sh2.Cells(1, k)) Then
                    columnIndex(j) = k
                    Exit For
               End If
            Next
        Next
        
        Dim isFinished As Boolean
        isFinished = False
        Dim a As Integer
        a = 2
        Do
            If IsEmpty(sh2.Cells(a, 2)) Then
                '判定该表已读完
                If IsEmpty(sh2.Cells(a + 1, 2)) And IsEmpty(sh2.Cells(a + 2, 2)) Then
                    isFinished = True
                End If
            Else
                For b = 2 To columnMax1
                    '表一没有的数据，表二仍为空
                    If IsEmpty(columnIndex(b)) Then
                        sh1.Cells(existingRows, b) = Empty
                    Else
                        sh1.Cells(existingRows, b) = sh2.Cells(a, columnIndex(b))
                    End If
                Next
            
                '添加组别名
                If IsEmpty(sh2.Cells(a, 1)) Then
                    sh1.Cells(existingRows, 1) = sh1.Cells(existingRows - 1, 1)
                Else
                    sh1.Cells(existingRows, 1) = sh2.Cells(a, 1)
                End If
                '计数，汇总表添加一行
                existingRows = existingRows + 1
            End If
            
            '读取目标表下一行
            a = a + 1
            
            '依据需求自行调整该段以判定下方是否还有有用数据
            '若无，则判定该表是否已完全转移完
            'If sh2.Cells(a, 1) = "" Then
            '    isFinished = True
            'End If
        Loop While isFinished = False
    Next

    sourcebook.Close
    ThisWorkbook.Save
    Application.ScreenUpdating = True

End Sub
```
