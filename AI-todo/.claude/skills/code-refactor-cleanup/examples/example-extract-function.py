from templates.template-refactor import extract_function

original = """
x = 10
y = 20
print(x + y)
"""

refactored = extract_function(original, "sum_numbers")
print(refactored)
