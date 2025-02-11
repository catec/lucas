# #!/bin/bash

# --------------- *.cpp & *.h ---------------
temp_file_cpp=$(mktemp)
cat <<EOL > "$temp_file_cpp"
//---------------------------------------------------------------------------------------------------------------------
//  LUCAS: Lightweight framework for UAV Control And Supervision
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
//---------------------------------------------------------------------------------------------------------------------
// This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
// version.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program. If not, see
// https://www.gnu.org/licenses/.
//---------------------------------------------------------------------------------------------------------------------

EOL

for archivo in $(find . -type f \( -name "*.cpp" -o -name "*.h" \)); do
  temp_combined=$(mktemp)
  cat "$temp_file_cpp" "$archivo" > "$temp_combined"
  mv "$temp_combined" "$archivo"
done
rm "$temp_file_cpp"

echo "Copyright added to all '.cpp' and '.h' files."


# ---------------------------------------------------------------------------------------------------------------------
# --------------- CMakeLists.txt ---------------
temp_file=$(mktemp)
cat <<EOL > "$temp_file"
##---------------------------------------------------------------------------------------------------------------------
##  LUCAS: Lightweight framework for UAV Control And Supervision
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
##---------------------------------------------------------------------------------------------------------------------
## This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
## License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
## version.
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
## You should have received a copy of the GNU General Public License along with this program. If not, see
## https://www.gnu.org/licenses/.
##---------------------------------------------------------------------------------------------------------------------

EOL

for archivo in $(find . -type f \( -name "CMakeLists.txt"  \)); do
  temp_combined=$(mktemp)
  cat "$temp_file" "$archivo" > "$temp_combined"
  mv "$temp_combined" "$archivo"
done

# Eliminar el archivo temporal
rm "$temp_file"

echo "Copyright added to all 'CMakeLists.txt' files."

# ---------------------------------------------------------------------------------------------------------------------
# # --------------- *.py ---------------
# The line we expect to find
EXPECTED_LINE="#!/usr/bin/env python3"

temp_file=$(mktemp)
cat <<EOL > "$temp_file"

##---------------------------------------------------------------------------------------------------------------------
##  LUCAS: Lightweight framework for UAV Control And Supervision
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
##---------------------------------------------------------------------------------------------------------------------
## This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
## License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
## version.
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
## You should have received a copy of the GNU General Public License along with this program. If not, see
## https://www.gnu.org/licenses/.
##---------------------------------------------------------------------------------------------------------------------

EOL

# Find and modify all .cpp and .h files in the current directory and its subdirectories
for file in $(find . -type f \( -name "*.py" \)); do
  # Check if the file contains the expected line
  if grep -q "$EXPECTED_LINE" "$file"; then
    # Insert the block of text right after the expected line
    awk -v expected_line="$EXPECTED_LINE" -v block="$(cat $temp_file)" '
    $0 == expected_line {
        print $0
        print block
        next
    }
    {print $0}' "$file" > "$file.tmp" && mv "$file.tmp" "$file"

    echo "Block of text added below the expected line in $file."
  else
    echo "Warning: $file does not contain the expected line. File not modified."
  fi
done

# Remove the temporary file
rm "$temp_file"

echo "Copyright added to all '*.py' files."


