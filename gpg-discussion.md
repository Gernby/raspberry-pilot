# GnuPG (GPG) discussion

## About

GnuPG (GPG) is a data encryption utility. It allows data to be encrypted in such a way that only specific people can decrypt it. It allows people who receive data to verify that it came from the person who claimed to send it.

## GPG Secret Key Generation

Create a file in your home directory with the following contents, make substitutions as appropriate:

%echo Generating a default key
Key-Type: default
Subkey-Type: default
Name-Real: \<your Discord nickname\>
Name-Comment: \<your car type\>
Name-Email: \<your email\>
Expire-Date: 1000
Passphrase: abc
# Do a commit here, so that we can later print "done" :-)
%commit
%echo done

Then at the command prompt run

`gpg --batch --generate-key <name of file with contents above>`

Wait 7 hours. When done, run

`gpg --export -a`

Copy and paste the output into a file called `<your name>.asc`. Submit a PR to add the output to the end of the file named for your car type, such as Honda Bosch.
