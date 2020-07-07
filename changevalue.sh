#! /bin/bash

# you must sudo apt install jq -y
# place the param name in /home/ubuntu/changeparam/param
# place the plus-minus in /home/ubuntu/changeparam/plus-minus
# place the delta in /home/ubuntu/changeparam/delta
# then call this script with sh /home/ubuntu/changeparam/changevalue.sh

# capture the current value of the parameter into oldvalue
cat /home/ubuntu/kegman.json | jq -r .`cat /home/ubuntu/changeparam/param` > /home/ubuntu/changeparam/oldvalue

# use bc to add (or subtract) the delta against the oldvalue and store as newvalue no quotes
echo `cat /home/ubuntu/changeparam/oldvalue``cat /home/ubuntu/changeparam/plus-minus``cat /home/ubuntu/changeparam/delta` | bc > /home/ubuntu/changeparam/newvalue

# rewrite the parameter with the new value, add quotes around it, and store in temp file
cat /home/ubuntu/kegman.json | jq ".`cat /home/ubuntu/changeparam/param` = `cat /home/ubuntu/changeparam/newvalue`" | jq '(..|select(type == "number")) |= tostring' > /home/ubuntu/kegman.json.tmp

# delete prior kegman backup, backup the current kegman, and overwrite the current kegman with the new value set
rm /home/ubuntu/kegman.json.old
cp /home/ubuntu/kegman.json /home/ubuntu/kegman.json.old
mv /home/ubuntu/kegman.json.tmp /home/ubuntu/kegman.json
rm /home/ubuntu/changeparam/oldvalue /home/ubuntu/changeparam/newvalue /home/ubuntu/changeparam/delta /home/ubuntu/changeparam/plus-minus

cat /home/ubuntu/changeparam/param && cat /home/ubuntu/kegman.json | jq .`cat /home/ubuntu/changeparam/param`
rm /home/ubuntu/changeparam/param
