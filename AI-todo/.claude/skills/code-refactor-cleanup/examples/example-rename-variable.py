from templates.template-refactor import rename_variable

code = "a = 5\nb = a + 10"
new_code = rename_variable(code, "a", "num")
print(new_code)
