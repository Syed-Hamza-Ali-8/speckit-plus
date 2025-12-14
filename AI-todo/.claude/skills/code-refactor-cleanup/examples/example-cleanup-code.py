from templates.template-cleanup import remove_unused_imports, format_code

code = """
import os
import sys

print('Hello World')
"""

code = remove_unused_imports(code)
code = format_code(code)
print(code)
