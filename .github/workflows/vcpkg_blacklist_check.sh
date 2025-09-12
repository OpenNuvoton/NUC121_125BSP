#!/bin/bash

BUILDLOG="$1"
TMPFILE=$(mktemp)
FOUND=0

# Define blacklist keywords to search for
BLACKLIST=(
    "[Fatal Error]"
    "An error has occurred."
    " error: "
    " warning: "
    " Error: "
    " Warning: "
    " Warning[ "
    "warning csolution: "
)

# Read the build log file line by line
while IFS= read -r line; do
    echo "$line" >> "$TMPFILE"
    for keyword in "${BLACKLIST[@]}"; do
        # Check if the line contains the blacklist keyword
        if [[ "$line" == *"$keyword"* ]]; then
            # Find the position of the keyword in the line (0-based index)
            pos=$(awk -v a="$line" -v b="$keyword" 'BEGIN{print index(a, b) - 1}')
            # Create a pointer line with spaces and '^' characters
            arrow=$(printf '%*s' "$pos" '' | tr ' ' ' ')
            arrow+="$(printf '%*s' "${#keyword}" '' | tr ' ' '^') <- Received a blacklist rule."
            # Append the pointer line
            echo "$arrow" >> "$TMPFILE"
            echo "" >> "$TMPFILE"
            # Increase the counter and break out of the loop
            FOUND=$((FOUND + 1))
            break
        fi
    done
done < "$BUILDLOG"

# If any blacklist rule was found, update the original log file
if [[ $FOUND -gt 0 ]]; then
    mv "$TMPFILE" "$BUILDLOG"
else
    rm "$TMPFILE"
fi

# Return number of violations as exit code
exit $FOUND
