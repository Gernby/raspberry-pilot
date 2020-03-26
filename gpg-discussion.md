# GnuPG (GPG) discussion

## About

GnuPG (GPG) is an encryption and validation utility based on Public Key Cryptography. It allows data to be encrypted in such a way that only specific people can decrypt it. It also allows people who receive data to verify that it came from the person who claimed to send it.

The installation instructions include a step to let Gernby know when you are ready to receive a model file. Raspberry Pilot will not be able to issue steering commands without that file. Once Gernby knows your car is ready to receive steering commands, he will send the file to you. With your public key, he could publish one file that only the people who are ready for the file could use it.

Below are the steps required to generate your secret and public keys and for sharing your public key. This will simplify the model distribution while maintaining protections that are still required at this stage. This is only a temporary measure and the restriction will be lifted once more cars are validated.

## GPG Secret Key Generation

Create a file in your home directory with the following contents, make substitutions as appropriate:

```
%echo Generating a default key  
Key-Type: default  
Subkey-Type: default  
Name-Real: \<your Discord nickname\>  
Name-Comment: \<your car type\>  
Name-Email: \<your email\>  
Expire-Date: 1000  
Passphrase: abc  
%commit  
%echo done  
```

Then at the command prompt run

`gpg --batch --generate-key <name of file with contents above>`

Wait 7 hours. When done, run

`gpg --export -a`

Copy and paste the output into a PR to add the output to the end of the file named for your car type, such as `Honda Bosch`.
