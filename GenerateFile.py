import xlrd

loc = ("output.xls")

# To open Workbook 
wb = xlrd.open_workbook(loc) 
wb_sheet = wb.sheet_by_index(0)


with open("pushups.txt", "w") as file:
    # For row 0 and column 0 
    for row_idx in range(1, wb_sheet.nrows):
        for col_idx in range(wb_sheet.ncols):
            file.write("%s\n" % wb_sheet.cell(row_idx, col_idx).value)
            
        
    
